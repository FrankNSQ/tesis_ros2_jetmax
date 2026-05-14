#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.color_pub = self.create_publisher(
            String,
            '/vision/detected_color',
            10
        )

        self.position_pub = self.create_publisher(
            String,
            '/vision/detected_cube',
            10
        )

        # Solo para no spamear logs
        self.last_logged = {
            'rojo': None,
            'verde': None,
            'azul': None
        }

        # ---------- CALIBRACIÓN DE LA HOJA ----------
        img_pts = np.array([
            [578, 6],       #Superior Izquierda
            [44, 7],      #Superior Derecha
            [41, 450],     #Superior derecha
            [583, 450],      #Superior izquierda
        ], dtype=np.float32)

        real_pts = np.array([
            [-100, 262],
            [100, 262],
            [100, 0],
            [-100, 0],
        ], dtype=np.float32)

        self.H = cv2.getPerspectiveTransform(img_pts, real_pts)

        self.get_logger().info('✅ Nodo color_detector iniciado')
        self.get_logger().info('Escuchando /image_raw')
        self.get_logger().info('Homografía cargada para convertir imagen -> hoja')
        self.get_logger().info('Umbrales de estabilidad: 4 px, 3.0 mm')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([35, 80, 80])
        upper_green = np.array([85, 255, 255])

        lower_blue = np.array([90, 80, 80])
        upper_blue = np.array([130, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        self.process_mask(frame, mask_red, 'rojo')
        self.process_mask(frame, mask_green, 'verde')
        self.process_mask(frame, mask_blue, 'azul')

        corners = [(578, 6), (44, 7), (41, 450), (583, 451)]
        for pt in corners:
            cv2.circle(frame, pt, 5, (0, 0, 255), -1)

        cv2.imshow('Deteccion de colores', frame)
        cv2.waitKey(1)

    def pixel_to_table_mm(self, cx, cy):
        pt = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pt, self.H)
        x_mm = float(mapped[0][0][0])
        y_mm = float(mapped[0][0][1])
        return x_mm, y_mm

    def process_mask(self, frame, mask, color_name):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        frame_h, frame_w = frame.shape[:2]
        best_detection = None
        best_area = 0

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 500 or area > 50000:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            if x <= 5 or y <= 5 or (x + w) >= (frame_w - 5) or (y + h) >= (frame_h - 5):
                continue

            aspect_ratio = w / float(h)
            if aspect_ratio < 0.7 or aspect_ratio > 1.3:
                continue

            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

            if len(approx) < 4 or len(approx) > 6:
                continue

            if area > best_area:
                best_area = area
                best_detection = (x, y, w, h)

        if best_detection is None:
            return

        x, y, w, h = best_detection
        cx = x + w // 2
        cy = y + h // 2

        x_mm, y_mm = self.pixel_to_table_mm(cx, cy)
        x_mm = round(x_mm, 1)
        y_mm = round(y_mm, 1)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

        text1 = f'{color_name} ({cx},{cy})'
        text2 = f'X={x_mm:.1f} mm  Y={y_mm:.1f} mm'

        cv2.putText(
            frame,
            text1,
            (x, y - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            text2,
            (x, y - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2
        )

        # Publicación continua para que cube_state_publisher siempre tenga datos
        msg1 = String()
        msg1.data = f'{color_name},{cx},{cy}'
        self.color_pub.publish(msg1)

        msg2 = String()
        msg2.data = f'{color_name},{cx},{cy},{x_mm:.1f},{y_mm:.1f}'
        self.position_pub.publish(msg2)

        # Solo log si cambió
        current_log = msg2.data
        if self.last_logged[color_name] != current_log:
            self.last_logged[color_name] = current_log
            self.get_logger().info(f'Detectado: {msg2.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()