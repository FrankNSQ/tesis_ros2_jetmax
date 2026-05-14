#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker


class DistanceMarker(Node):
    def __init__(self):
        super().__init__('distance_marker')

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        self.marker_pub = self.create_publisher(Marker, '/distance_marker', 10)

        self.points = []

        self.get_logger().info('✅ Nodo distance_marker listo')
        self.get_logger().info('Usa la herramienta "Publish Point" en RViz')
        self.get_logger().info('Selecciona 2 puntos para medir la distancia')

    def point_callback(self, msg: PointStamped):
        p = Point()
        p.x = msg.point.x
        p.y = msg.point.y
        p.z = msg.point.z

        self.points.append(p)

        # Mantener solo los últimos 2 puntos
        if len(self.points) > 2:
            self.points.pop(0)

        self.publish_markers(msg.header.frame_id)

        if len(self.points) == 2:
            d = self.distance(self.points[0], self.points[1])
            self.get_logger().info(f' Distancia: {d:.4f} m ({d*100:.2f} cm)')

    def distance(self, p1: Point, p2: Point) -> float:
        return math.sqrt(
            (p2.x - p1.x) ** 2 +
            (p2.y - p1.y) ** 2 +
            (p2.z - p1.z) ** 2
        )

    def publish_markers(self, frame_id: str):
        if len(self.points) >= 1:
            self.publish_sphere(self.points[0], frame_id, 0, 0.0, 1.0, 0.0)

        if len(self.points) >= 2:
            self.publish_sphere(self.points[1], frame_id, 1, 0.0, 0.0, 1.0)
            self.publish_line(self.points[0], self.points[1], frame_id, 2)
            self.publish_text(self.points[0], self.points[1], frame_id, 3)

    def publish_sphere(self, point: Point, frame_id: str, marker_id: int, r: float, g: float, b: float):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'distance_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = point
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.015
        marker.scale.y = 0.015
        marker.scale.z = 0.015

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def publish_line(self, p1: Point, p2: Point, frame_id: str, marker_id: int):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'distance_line'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.005

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0
        marker.points = [p1, p2]

        self.marker_pub.publish(marker)

    def publish_text(self, p1: Point, p2: Point, frame_id: str, marker_id: int):
        d = self.distance(p1, p2)

        mid = Point()
        mid.x = (p1.x + p2.x) / 2.0
        mid.y = (p1.y + p2.y) / 2.0
        mid.z = (p1.z + p2.z) / 2.0 + 0.03

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'distance_text'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position = mid
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.025

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.text = f'{d:.4f} m\n{d*100:.2f} cm'

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = DistanceMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
