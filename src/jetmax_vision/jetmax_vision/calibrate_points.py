#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CalibratePoints(Node):
    def __init__(self):
        super().__init__('calibrate_points')

        self.bridge = CvBridge()
        self.frame = None
        self.points = []

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        cv2.namedWindow('Calibracion')
        cv2.setMouseCallback('Calibracion', self.mouse_callback)

        self.get_logger().info('✅ Nodo calibrate_points iniciado')
        self.get_logger().info('Haz clic en las 4 esquinas de la hoja en este orden:')
        self.get_logger().info('1) superior izquierda')
        self.get_logger().info('2) superior derecha')
        self.get_logger().info('3) inferior derecha')
        self.get_logger().info('4) inferior izquierda')

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        display = self.frame.copy()

        for i, pt in enumerate(self.points):
            cv2.circle(display, pt, 5, (0, 0, 255), -1)
            cv2.putText(
                display,
                f'{i+1}:{pt}',
                (pt[0] + 10, pt[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1
            )

        cv2.imshow('Calibracion', display)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append((x, y))
            self.get_logger().info(f'Punto {len(self.points)}: ({x}, {y})')

            if len(self.points) == 4:
                self.get_logger().info('✅ Ya tienes los 4 puntos:')
                for i, pt in enumerate(self.points):
                    self.get_logger().info(f'{i+1}: {pt}')
                self.get_logger().info('Copia estos puntos en img_pts dentro de color_detector.py')


def main(args=None):
    rclpy.init(args=args)
    node = CalibratePoints()

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

