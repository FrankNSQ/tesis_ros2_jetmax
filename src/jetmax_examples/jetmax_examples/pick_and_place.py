#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place')
        self.arm_pub = self.create_publisher(Point, '/arm/set_position', 10)
        self.suction_pub = self.create_publisher(Bool, '/arm/suction', 10)
        self.get_logger().info('✅ Nodo Pick & Place iniciado')

    def mover(self, x, y, z, segundos):
        punto = Point()
        punto.x = float(x)
        punto.y = float(y)
        punto.z = float(z)
        self.arm_pub.publish(punto)
        self.get_logger().info(f'➡️ Mover a x={x} y={y} z={z}')
        time.sleep(segundos)

    def succion(self, activar):
        msg = Bool()
        msg.data = activar
        self.suction_pub.publish(msg)
        estado = 'ON' if activar else 'OFF'
        self.get_logger().info(f'Succión {estado}')
        time.sleep(1.0)

    def ejecutar(self):
        self.get_logger().info('INICIANDO Pick & Place...')

        # HOME
        self.mover(0, -163, 212, 2.0)

        # Ir sobre la caja
        self.mover(0, -175, 160, 2.0)

        # Bajar a recoger
        self.mover(0, -175, 113, 2.0)

        # Activar succión: en simulación de la caja se adhire al efector final 
        self.succion(True)
        time.sleep(1.5)

        # Levantar
        self.mover(0, -170, 230, 2.0)

        # Mover al destino
        self.mover(80, -160, 210, 2.0)

        # Bajar al destino
        self.mover(80, -165, 113, 2.0)

        # Desactivar succión: en simulación se libera la caja en el destino
        self.succion(False)

        # Subir
        self.mover(80, -160, 210, 2.0)

        # Volver a HOME
        self.mover(0, -163, 212, 2.0)

        self.get_logger().info('✅ Pick & Place completado')


def main():
    rclpy.init()
    nodo = PickAndPlace()
    try:
        nodo.ejecutar()
    except KeyboardInterrupt:
        nodo.get_logger().warn('⚠️ Interrumpido por usuario')
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()