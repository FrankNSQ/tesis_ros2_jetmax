#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class GazeboCubeSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_cube_spawner')

        self.subscription = self.create_subscription(
            String,
            '/vision/cube_state',
            self.callback,
            10
        )

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.gazebo_state_pub = self.create_publisher(
            String,
            '/gazebo_cube_state_raw',
            latched_qos
        )

        self.pending_state = None
        self.last_synced_state = None
        self.busy = False

        self.cube_size = 0.04

        self.sheet_origin_x = 0.00
        self.sheet_origin_y = -0.09
        
        self.x_scale = 1.10
        self.y_scale = 0.70

        self.timer = self.create_timer(0.5, self.process_pending_state)

        self.get_logger().info('✅ Nodo gazebo_cube_spawner iniciado')
        self.get_logger().info('Escuchando /vision/cube_state')
        self.get_logger().info('Sincronizando cubos en Gazebo con SDF')
        self.get_logger().info('Publicando coordenadas exactas en /gazebo_cube_state')

    def callback(self, msg: String):
        state = msg.data.strip()
        if not state:
            return
        self.pending_state = state

    def process_pending_state(self):
        if self.busy:
            return
        if self.pending_state is None:
            return
        if self.pending_state == self.last_synced_state:
            return

        self.busy = True
        try:
            state = self.pending_state
            self.get_logger().info(f'Nuevo estado recibido: {state}')
            cubes = self.parse_state(state)
            gazebo_cubes = self.sync_cubes(cubes)
            if gazebo_cubes:
                self.publish_gazebo_state(gazebo_cubes)
            self.last_synced_state = state
        finally:
            self.busy = False

    def parse_state(self, state_str):
        cubes = []
        for entry in state_str.split(';'):
            parts = entry.split(',')
            if len(parts) != 3:
                continue
            color = parts[0].strip()
            x_mm = float(parts[1])
            y_mm = float(parts[2])
            cubes.append((color, x_mm, y_mm))
        return cubes

    def mm_to_gazebo(self, x_mm, y_mm):
        x_m = self.sheet_origin_x + self.x_scale * (x_mm / 1000.0)
        y_m = self.sheet_origin_y - self.y_scale * (y_mm / 1000.0)
        z_m = self.cube_size / 2.0
        return x_m, y_m, z_m

    def build_cube_sdf(self, color_name, model_name):
        rgba = {
            'rojo': '1 0 0 1',
            'verde': '0 1 0 1',
            'azul': '0 0 1 1'
        }.get(color_name, '0.6 0.6 0.6 1')

        size = f'{self.cube_size} {self.cube_size} {self.cube_size}'

        sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <pose>0 0 0 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.0000033333</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.0000033333</iyy>
          <iyz>0.0</iyz>
          <izz>0.0000033333</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size}</size>
          </box>
        </geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        return sdf

    def wait_for_services(self):
        if not self.spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity no disponible')
            return False
        if not self.delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity no disponible')
            return False
        return True

    def delete_cube_if_exists(self, name):
        req = DeleteEntity.Request()
        req.name = name
        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def spawn_cube(self, color, x_m, y_m, z_m):
        entity_name = f'{color}_cube_detected'
        sdf = self.build_cube_sdf(color, entity_name)

        req = SpawnEntity.Request()
        req.name = entity_name
        req.xml = sdf
        req.robot_namespace = entity_name
        req.initial_pose.position.x = x_m
        req.initial_pose.position.y = y_m
        req.initial_pose.position.z = z_m
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        result = future.result()
        if result is None:
            self.get_logger().warn(f'⚠️ Timeout al spawnear {entity_name}, pero puede haberse creado igual')
            return

        if getattr(result, 'success', False):
            self.get_logger().info(
                f'Spawn OK: {entity_name} en ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})'
            )
        else:
            self.get_logger().warn(
                f'⚠️ Spawn respondió sin éxito: {entity_name} | {getattr(result, "status_message", "")}'
            )

    def sync_cubes(self, cubes):
        if not self.wait_for_services():
            return []

        for color in ['rojo', 'verde', 'azul']:
            self.delete_cube_if_exists(f'{color}_cube_detected')

        time.sleep(0.4)

        gazebo_cubes = []
        for color, x_mm, y_mm in cubes:
            x_m, y_m, z_m = self.mm_to_gazebo(x_mm, y_mm)
            self.spawn_cube(color, x_m, y_m, z_m)
            gazebo_cubes.append((color, x_m, y_m, z_m))

        return gazebo_cubes

    def publish_gazebo_state(self, gazebo_cubes):
        entries = []
        for color, x_m, y_m, z_m in gazebo_cubes:
            entries.append(f'{color},{x_m:.3f},{y_m:.3f},{z_m:.3f}')

        msg = String()
        msg.data = ';'.join(entries)
        self.gazebo_state_pub.publish(msg)
        self.get_logger().info(f'Estado Gazebo publicado: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCubeSpawner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn('⚠️ Nodo interrumpido')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()