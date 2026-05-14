#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class Saludo(Node):
    def __init__(self):
        super().__init__('saludo')
        self.publisher = self.create_publisher(Point, '/arm/set_position', 10)
        self.get_logger().info('✅ Nodo Saludo iniciado')

    def mover(self, x, y, z, segundos):
        punto = Point()
        punto.x = float(x)
        punto.y = float(y)
        punto.z = float(z)
        self.publisher.publish(punto)
        self.get_logger().info(f'x={x} y={y} z={z}')
        time.sleep(segundos)

    def ejecutar(self):
        self.get_logger().info('INICIANDO saludo...')

        # Home
        self.mover(0, -163, 212, 2.0)

        # Subir brazo
        self.mover(0, -130, 235, 2.0)

        # Saludo derecha
        self.mover(65, -130, 235, 1.5)

        # Saludo izquierda
        self.mover(-70, -130, 235, 1.5)

        # Saludo derecha
        self.mover(65, -130, 235, 1.5)
        # Centro
        self.mover(0, -130, 235, 1.5)

        # Volver a home
        self.mover(0, -163, 212, 2.0)

        self.get_logger().info('✅ Saludo completado')


def main():
    rclpy.init()
    nodo = Saludo()
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        nodo.get_logger().warn('⚠️ Interrumpido por usuario')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()