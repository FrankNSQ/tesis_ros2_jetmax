#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration


class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.declare_parameter('fixed_frame', 'Base')
        self.declare_parameter('target_frame', 'virtual_end_effector')

        self.fixed_frame = self.get_parameter('fixed_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, '/jetmax_path', 10)

        self.points = []
        self.max_points = 2000

        self.timer = self.create_timer(0.1, self.update_path)

        self.get_logger().info(f'✅ Visualizador de trayectoria listo')
        self.get_logger().info(f'Frame fijo: {self.fixed_frame}')
        self.get_logger().info(f'Frame objetivo: {self.target_frame}')

    def update_path(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )

            p = Point()
            p.x = tf.transform.translation.x
            p.y = tf.transform.translation.y
            p.z = tf.transform.translation.z

            if not self.points:
                self.points.append(p)
            else:
                last = self.points[-1]
                dist = ((p.x - last.x) ** 2 + (p.y - last.y) ** 2 + (p.z - last.z) ** 2) ** 0.5
                if dist > 0.001:
                    self.points.append(p)

            if len(self.points) > self.max_points:
                self.points.pop(0)

            marker = Marker()
            marker.header.frame_id = self.fixed_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'jetmax_path'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.005

            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0

            marker.pose.orientation.w = 1.0
            marker.points = self.points

            self.marker_pub.publish(marker)

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

