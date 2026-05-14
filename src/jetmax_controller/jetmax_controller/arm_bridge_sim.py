#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from . import jetmax_kinematics


class ArmBridgeSim(Node):

    def __init__(self):
        super().__init__('arm_bridge_sim')

        self.subscription = self.create_subscription(
            Point,
            '/arm/set_position',
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'Joint1', 'Joint2', 'Joint3',
            'Joint4', 'Joint5', 'Joint6',
            'Joint7', 'Joint8', 'Joint9'
        ]

        self.get_logger().info("✅ Arm Bridge SIM listo")

    def callback(self, msg):

        x, y, z = -msg.x, msg.y, msg.z

        active = jetmax_kinematics.inverse_kinematics((x, y, z))
        if active is None:
            self.get_logger().error("IK falló")
            return

        joints_deg = jetmax_kinematics.forward_kinematics(active)
        if joints_deg is None:
            self.get_logger().error("FK falló")
            return

        joints_rad = [math.radians(a) for a in joints_deg]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joints_rad
        point.time_from_start = Duration(sec=2)

        traj.points.append(point)

        self.publisher.publish(traj)

        self.get_logger().info(f"➡️ Moviendo a {x}, {y}, {z}")


def main():
    rclpy.init()
    node = ArmBridgeSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()