#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class Exploracion(Node):
    def __init__(self):
        super().__init__('exploracion')
        self.publisher = self.create_publisher(Point, '/arm/set_position', 10)
        self.get_logger().info('✅ Nodo Exploración iniciado')

    def mover(self, x, y, z, segundos):
        punto = Point()
        punto.x = float(x)
        punto.y = float(y)
        punto.z = float(z)
        self.publisher.publish(punto)
        self.get_logger().info(f'➡️ x={x} y={y} z={z}')
        time.sleep(segundos)

    def ejecutar(self):
        self.get_logger().info('INICIANDO Exploración (3 posiciones)...')

        # HOME inicial
        self.mover(0, -163, 212, 2.0)

        # Posición A
        self.get_logger().info('Posición A: Derecha')
        self.mover(100, -160, 200, 2.0)

        # Posición B
        self.get_logger().info('Posición B: Izquierda')
        self.mover(-100, -160, 200, 2.0)

        # Posición C
        self.get_logger().info('Posición C: Centro bajo')
        self.mover(0, -170, 140, 2.0)

        # Volver a HOME
        self.get_logger().info('Regresando a HOME')
        self.mover(0, -163, 212, 2.0)

        self.get_logger().info('✅ Exploración completada')


def main():
    rclpy.init()
    nodo = Exploracion()
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        nodo.get_logger().warn('⚠️ Interrumpido por usuario')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
