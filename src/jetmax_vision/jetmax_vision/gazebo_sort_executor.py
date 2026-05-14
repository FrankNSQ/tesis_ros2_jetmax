#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from linkattacher_msgs.srv import AttachLink, DetachLink


class GazeboSortExecutor(Node):
    def __init__(self):
        super().__init__('gazebo_sort_executor')

        self.arm_pub = self.create_publisher(Point, '/arm/set_position', 10)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            String,
            '/gazebo_cube_state',
            self.state_callback,
            latched_qos
        )

        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        self.latest_state = None
        self.state_received = False

        self.robot_model = 'jetmax'
        self.robot_link = 'Link8'

        self.sort_order = ['rojo', 'verde', 'azul']

        # Ahora sí activamos succión
        self.dry_run = False

        # Zonas finales en Gazebo
        self.goal_gazebo = {
            'rojo':  (-0.10, -0.12, 0.02),
            'verde': ( 0.00, -0.12, 0.02),
            'azul':  ( 0.10, -0.12, 0.02),
        }

        self.z_safe = 210.0
        self.z_pick = 118.0
        self.z_place = 120.0
        self.wait_move = 2.0

        self.get_logger().info('✅ Nodo gazebo_sort_executor iniciado')
        self.get_logger().info('Esperando /gazebo_cube_state')
        self.get_logger().info(f'Orden: {self.sort_order}')
        self.get_logger().info('Succión real activada')

    def state_callback(self, msg: String):
        self.latest_state = msg.data
        self.state_received = True
        self.get_logger().info(f'Snapshot Gazebo recibido: {msg.data}')

    def parse_state(self, state_str):
        cubes = {}
        if not state_str:
            return cubes

        for entry in state_str.split(';'):
            parts = entry.split(',')
            if len(parts) != 4:
                continue

            color = parts[0].strip()
            x_m = float(parts[1])
            y_m = float(parts[2])
            z_m = float(parts[3])
            cubes[color] = (x_m, y_m, z_m)

        return cubes

    def gazebo_to_arm(self, x_m, y_m):
        """
        Conversión Gazebo -> /arm/set_position
        teniendo en cuenta que arm_bridge_sim invierte X internamente.
        """
        x_cmd = -(937.5 * x_m)
        y_arm = 920.0 * y_m

        x_cmd = max(min(x_cmd, 100.0), -100.0)
        y_arm = max(min(y_arm, -80.0), -261.0)

        return x_cmd, y_arm

    def move_to(self, x, y, z, seconds=None):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        self.arm_pub.publish(p)
        self.get_logger().info(f'➡️ Mover a x={x:.1f} y={y:.1f} z={z:.1f}')
        time.sleep(seconds if seconds is not None else self.wait_move)

    def call_attach(self, cube_color):
        if not self.attach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/ATTACHLINK no disponible')
            return False

        req = AttachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = f'{cube_color}_cube_detected'
        req.link2_name = 'link'

        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)

        result = future.result()
        if result is None:
            self.get_logger().error(f'Timeout en ATTACHLINK para {cube_color}')
            return False

        if result.success:
            self.get_logger().info(f'ATTACH OK: {cube_color}')
            return True

        self.get_logger().error(f'ATTACH falló: {result.message}')
        return False

    def call_detach(self, cube_color):
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/DETACHLINK no disponible')
            return False

        req = DetachLink.Request()
        req.model1_name = self.robot_model
        req.link1_name = self.robot_link
        req.model2_name = f'{cube_color}_cube_detected'
        req.link2_name = 'link'

        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)

        result = future.result()
        if result is None:
            self.get_logger().error(f'Timeout en DETACHLINK para {cube_color}')
            return False

        if result.success:
            self.get_logger().info(f'DETACH OK: {cube_color}')
            return True

        self.get_logger().error(f'DETACH falló: {result.message}')
        return False

    def is_near_goal(self, color, x_m, y_m):
        gx, gy, _ = self.goal_gazebo[color]
        dist = math.sqrt((x_m - gx) ** 2 + (y_m - gy) ** 2)
        return dist < 0.03

    def sort_one_cube(self, color, x_m, y_m, z_m):
        self.get_logger().info(
            f'Ordenando {color} desde Gazebo ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})'
        )

        if self.is_near_goal(color, x_m, y_m):
            self.get_logger().info(f'{color} ya está cerca de su zona final')
            return True

        src_x, src_y = self.gazebo_to_arm(x_m, y_m)
        dst_x_m, dst_y_m, _ = self.goal_gazebo[color]
        dst_x, dst_y = self.gazebo_to_arm(dst_x_m, dst_y_m)

        # pick
        self.move_to(src_x, src_y, self.z_safe, 2.0)
        self.move_to(src_x, src_y, self.z_pick, 2.0)

        if self.dry_run:
            self.get_logger().info('DRY RUN activo')
            self.move_to(src_x, src_y, self.z_safe, 2.0)
            return True

        if not self.call_attach(color):
            self.get_logger().error(f'No se pudo adherir {color}')
            self.move_to(src_x, src_y, self.z_safe, 2.0)
            return False

        time.sleep(1.0)
        self.move_to(src_x, src_y, self.z_safe, 2.0)

        # place
        self.move_to(dst_x, dst_y, self.z_safe, 2.0)
        self.move_to(dst_x, dst_y, self.z_place, 2.0)

        if not self.call_detach(color):
            self.get_logger().error(f'No se pudo liberar {color}')
            self.move_to(dst_x, dst_y, self.z_safe, 2.0)
            return False

        time.sleep(1.0)
        self.move_to(dst_x, dst_y, self.z_safe, 2.0)

        self.get_logger().info(f'✅ {color} ordenado')
        return True

    def run_once(self):
        t0 = time.time()
        while rclpy.ok() and not self.state_received:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > 10.0:
                self.get_logger().error('No se recibió /gazebo_cube_state')
                return

        cubes = self.parse_state(self.latest_state)
        if not cubes:
            self.get_logger().error('Snapshot vacío')
            return

        self.get_logger().info(f'Cubos disponibles: {list(cubes.keys())}')

        self.move_to(0.0, -163.0, 212.0, 2.0)

        for color in self.sort_order:
            if color not in cubes:
                self.get_logger().warn(f'{color} no está en la escena')
                continue

            x_m, y_m, z_m = cubes[color]
            ok = self.sort_one_cube(color, x_m, y_m, z_m)
            if not ok:
                break

        self.move_to(0.0, -163.0, 212.0, 2.0)
        self.get_logger().info('Ejecución finalizada')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSortExecutor()

    try:
        node.run_once()
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()