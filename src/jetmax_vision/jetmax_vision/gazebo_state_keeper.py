#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


class GazeboStateKeeper(Node):
    def __init__(self):
        super().__init__('gazebo_state_keeper')

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            String,
            '/gazebo_cube_state_raw',
            self.callback,
            latched_qos
        )

        self.publisher = self.create_publisher(
            String,
            '/gazebo_cube_state',
            latched_qos
        )

        self.last_msg = None

        self.get_logger().info('✅ Nodo gazebo_state_keeper iniciado')
        self.get_logger().info('Escuchando /gazebo_cube_state_raw')
        self.get_logger().info('Republicando estado persistente en /gazebo_cube_state')

    def callback(self, msg: String):
        self.last_msg = msg.data
        out = String()
        out.data = self.last_msg
        self.publisher.publish(out)
        self.get_logger().info(f'Estado guardado: {out.data}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboStateKeeper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
