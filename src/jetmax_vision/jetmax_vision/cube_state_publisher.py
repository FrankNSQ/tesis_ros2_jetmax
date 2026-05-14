#!/usr/bin/env python3

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class CubeStatePublisher(Node):
    def __init__(self):
        super().__init__('cube_state_publisher')

        self.subscription = self.create_subscription(
            String,
            '/vision/detected_cube',
            self.callback,
            10
        )

        # QoS latched para que nuevos suscriptores reciban el último estado
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher = self.create_publisher(
            String,
            '/vision/cube_state',
            latched_qos
        )

        # Estado confirmado actual
        self.confirmed_state = {
            'rojo': None,
            'verde': None,
            'azul': None
        }

        # Buffers temporales para confirmar nueva posición
        self.buffers = {
            'rojo': deque(maxlen=8),
            'verde': deque(maxlen=8),
            'azul': deque(maxlen=8)
        }

        self.last_seen_time = {
            'rojo': 0.0,
            'verde': 0.0,
            'azul': 0.0
        }

        self.timeout_sec = 5.0
        self.publish_period = 1.0
        self.quantization_mm = 10.0
        self.min_samples = 5
        self.max_spread_mm = 8.0
        self.reconfirm_move_mm = 15.0

        self.last_published_msg = None

        self.timer = self.create_timer(self.publish_period, self.publish_state)

        self.get_logger().info('✅ Nodo cube_state_publisher iniciado')
        self.get_logger().info('Escuchando /vision/detected_cube')
        self.get_logger().info('Publicando estado completo en /vision/cube_state')
        self.get_logger().info(f'Timeout de persistencia: {self.timeout_sec} s')
        self.get_logger().info(f'Cuantización del estado: {self.quantization_mm} mm')
        self.get_logger().info(f'Muestras mínimas para confirmar: {self.min_samples}')

    def quantize(self, x_mm, y_mm):
        qx = round(x_mm / self.quantization_mm) * self.quantization_mm
        qy = round(y_mm / self.quantization_mm) * self.quantization_mm
        return round(qx, 1), round(qy, 1)

    def callback(self, msg: String):
        try:
            parts = msg.data.split(',')
            color = parts[0]
            x_mm = float(parts[3])
            y_mm = float(parts[4])
        except Exception as e:
            self.get_logger().warn(f'⚠️ Mensaje inválido: {msg.data} | Error: {e}')
            return

        if color not in self.confirmed_state:
            return

        qx, qy = self.quantize(x_mm, y_mm)

        self.buffers[color].append((qx, qy))
        self.last_seen_time[color] = time.time()

        self.try_confirm_color(color)

    def try_confirm_color(self, color):
        buffer = self.buffers[color]

        if len(buffer) < self.min_samples:
            return

        avg_x = sum(p[0] for p in buffer) / len(buffer)
        avg_y = sum(p[1] for p in buffer) / len(buffer)

        spread = max(
            math.sqrt((p[0] - avg_x) ** 2 + (p[1] - avg_y) ** 2)
            for p in buffer
        )

        if spread > self.max_spread_mm:
            return

        candidate = self.quantize(avg_x, avg_y)
        current = self.confirmed_state[color]

        if current is None:
            self.confirmed_state[color] = candidate
            self.get_logger().info(
                f'Confirmado inicial: {color} -> ({candidate[0]:.1f},{candidate[1]:.1f}) | spread={spread:.2f}'
            )
            return

        dist = math.sqrt(
            (candidate[0] - current[0]) ** 2 +
            (candidate[1] - current[1]) ** 2
        )

        if dist >= self.reconfirm_move_mm:
            self.confirmed_state[color] = candidate
            self.get_logger().info(
                f'Nueva posición confirmada: {color} -> ({candidate[0]:.1f},{candidate[1]:.1f}) | spread={spread:.2f}'
            )

    def publish_state(self):
        now = time.time()
        entries = []

        for color in ['rojo', 'verde', 'azul']:
            pos = self.confirmed_state[color]
            if pos is None:
                continue

            age = now - self.last_seen_time[color]
            if age > self.timeout_sec:
                continue

            x_mm, y_mm = pos
            entries.append(f'{color},{x_mm:.1f},{y_mm:.1f}')

        if not entries:
            return

        msg = String()
        msg.data = ';'.join(entries)

        # Re-publicar siempre el estado actual para robustez,
        # pero solo loggear cuando cambia
        self.publisher.publish(msg)

        if msg.data != self.last_published_msg:
            self.last_published_msg = msg.data
            self.get_logger().info(f'Estado publicado: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CubeStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()