#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from linkattacher_msgs.srv import AttachLink, DetachLink
from tf2_ros import Buffer, TransformListener


class SuctionAttachSim(Node):
    def __init__(self):
        super().__init__('suction_attach_sim')

        self.subscription = self.create_subscription(
            Bool,
            '/arm/suction',
            self.callback,
            10
        )

        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        self.attached = False
        self.pending_attach = False
        self.pending_detach = False
        self.busy = False

        self.robot_model = 'jetmax'
        self.robot_link = 'Link8'
        self.box_model = 'red_box'
        self.box_link = 'red_box_link'

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Posición conocida de la caja
        self.box_x = 0.0
        self.box_y = -0.185
        self.box_z = 0.03

        # Offset del punto real de succión respecto a Link8
        self.ee_offset_x = 0.01
        self.ee_offset_y = -0.017
        self.ee_offset_z = -0.075

        # Umbral de cercanía
        self.attach_distance_threshold = 0.04

        self.timer = self.create_timer(0.2, self.process_actions)

        self.get_logger().info('✅ Suction Attach SIM listo')
        self.get_logger().info('Escuchando /arm/suction')
        self.get_logger().info(
            f' Configuración: {self.robot_model}::{self.robot_link} <-> {self.box_model}::{self.box_link}'
        )
        self.get_logger().info(
            f'Caja esperada en: x={self.box_x}, y={self.box_y}, z={self.box_z}'
        )
        self.get_logger().info(
            f'Umbral de attach: {self.attach_distance_threshold} m'
        )

    def callback(self, msg: Bool):
        if msg.data and not self.attached:
            self.pending_attach = True
            self.pending_detach = False
            self.get_logger().info('Solicitud de ATTACH recibida')
        elif (not msg.data) and self.attached:
            self.pending_detach = True
            self.pending_attach = False
            self.get_logger().info('Solicitud de DETACH recibida')

    def process_actions(self):
        if self.busy:
            return

        if self.pending_attach:
            self.pending_attach = False
            self.attach_box_if_close()

        elif self.pending_detach:
            self.pending_detach = False
            self.detach_box()

    def get_link8_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                'Link8',
                rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f'⚠️ No se pudo obtener TF de Link8: {e}')
            return None

    def get_effective_suction_point(self):
        tf = self.get_link8_transform()
        if tf is None:
            return None

        # Aproximación suficiente para este caso:
        # usamos la traslación de Link8 + offset local del efector
        x = tf.transform.translation.x + self.ee_offset_x
        y = tf.transform.translation.y + self.ee_offset_y
        z = tf.transform.translation.z + self.ee_offset_z

        return x, y, z

    def compute_distance_to_box(self):
        suction_point = self.get_effective_suction_point()
        if suction_point is None:
            return None

        sx, sy, sz = suction_point
        d = math.sqrt(
            (sx - self.box_x) ** 2 +
            (sy - self.box_y) ** 2 +
            (sz - self.box_z) ** 2
        )

        self.get_logger().info(
            f'Distancia succión-caja: {d:.4f} m | Succión=({sx:.3f}, {sy:.3f}, {sz:.3f})'
        )
        return d

    def attach_box_if_close(self):
        distance = self.compute_distance_to_box()
        if distance is None:
            self.get_logger().error('No se pudo calcular la distancia a la caja')
            return

        if distance > self.attach_distance_threshold:
            self.get_logger().warn(
                f'⚠️ Caja demasiado lejos para attach ({distance:.4f} m > {self.attach_distance_threshold} m)'
            )
            return

        self.attach_box()

    def attach_box(self):
        if not self.attach_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Servicio /ATTACHLINK no disponible')
            return

        req = AttachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = self.box_model
        req.link2_name = self.box_link

        self.get_logger().info(
            f'Intentando ATTACH: {self.robot_model}::{self.robot_link} <-> {self.box_model}::{self.box_link}'
        )

        self.busy = True
        future = self.attach_client.call_async(req)
        future.add_done_callback(self.attach_done)

    def attach_done(self, future):
        self.busy = False
        try:
            result = future.result()
            self.get_logger().info(f'Respuesta ATTACHLINK: {result}')
            if getattr(result, 'success', False):
                self.attached = True
                self.get_logger().info('Caja adherida al efector final')
            else:
                self.get_logger().error('ATTACHLINK respondió success=False')
        except Exception as e:
            self.get_logger().error(f'Excepción en ATTACHLINK: {e}')

    def detach_box(self):
        if not self.detach_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('ervicio /DETACHLINK no disponible')
            return

        req = DetachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = self.box_model
        req.link2_name = self.box_link

        self.get_logger().info(
            f'Intentando DETACH: {self.robot_model}::{self.robot_link} <-> {self.box_model}::{self.box_link}'
        )

        self.busy = True
        future = self.detach_client.call_async(req)
        future.add_done_callback(self.detach_done)

    def detach_done(self, future):
        self.busy = False
        try:
            result = future.result()
            self.get_logger().info(f'Respuesta DETACHLINK: {result}')
            if getattr(result, 'success', False):
                self.attached = False
                self.get_logger().info('Caja liberada')
            else:
                self.get_logger().error('DETACHLINK respondió success=False')
        except Exception as e:
            self.get_logger().error(f'Excepción en DETACHLINK: {e}')


def main():
    rclpy.init()
    node = SuctionAttachSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()