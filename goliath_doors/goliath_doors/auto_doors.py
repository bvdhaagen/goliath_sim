#!/usr/bin/env python3
"""
Automatic sliding doors.

Spawns a kinematic sliding panel in each wall opening, then continuously:
  * reads the robot's world position (gazebo /get_entity_state),
  * for every door, if the robot is within `open_distance` it slides the door
    OPEN (into the adjacent wall); beyond `close_distance` it slides it CLOSED,
  * moves the door via gazebo /set_entity_state.

Doors are kinematic (movable, never fall, still block the robot, and Gazebo
syncs their motion smoothly). Spawns are synchronous (so the loop never races a
not-yet-spawned door) and the robot-pose poll is self-healing (so it can't
freeze and leave a door stuck open). Hysteresis (open<close) prevents flicker.
"""
import math
import os

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, GetEntityState, SetEntityState


DOOR_SDF = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link">
      <kinematic>true</kinematic>
      <inertial>
        <mass>50.0</mass>
        <inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
      <visual name="v">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular></material>
      </visual>
    </link>
  </model>
</sdf>"""


class AutoDoors(Node):
    def __init__(self):
        super().__init__('auto_doors')

        self.declare_parameter('config', '')
        cfg_path = self.get_parameter('config').value
        if not cfg_path or not os.path.exists(cfg_path):
            raise RuntimeError(f'doors config not found: {cfg_path!r}')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.robot_model = cfg.get('robot_model', 'goliath')
        self.open_dist = float(cfg.get('open_distance', 3.0))
        self.close_dist = float(cfg.get('close_distance', self.open_dist * 1.5))
        self.slide_speed = float(cfg.get('slide_speed', 2.5))
        self.rate = float(cfg.get('rate', 30.0))
        self.color = cfg.get('color', [0.25, 0.35, 0.55])
        self.doors = cfg['doors']
        self.dt = 1.0 / self.rate

        self.progress = {n: 0.0 for n in self.doors}
        self.target = {n: 0.0 for n in self.doors}
        self.robot_xy = None
        self._robot_future = None
        self._ticks_without_robot = 0

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_cli = self.create_client(GetEntityState, '/get_entity_state')
        self.set_cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info('Waiting for gazebo spawn/get/set services...')
        self.spawn_cli.wait_for_service()
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()

        for name in self.doors:                # spawn synchronously (wait each)
            self._spawn_door(name)

        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'Auto-doors ready: {list(self.doors)} '
            f'(open within {self.open_dist} m of {self.robot_model}).')

    def _door_pose(self, name, progress):
        c = self.doors[name]['closed']
        s = self.doors[name]['slide']
        return (c[0] + s[0] * progress, c[1] + s[1] * progress, c[2] + s[2] * progress)

    def _spawn_door(self, name):
        d = self.doors[name]
        x, y, z = d['closed']
        sx, sy, sz = d['size']
        req = SpawnEntity.Request()
        req.name = name
        req.xml = DOOR_SDF.format(name=name, x=x, y=y, z=z, sx=sx, sy=sy, sz=sz,
                                  r=self.color[0], g=self.color[1], b=self.color[2])
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            self.get_logger().warn(f'spawn {name}: no response (timeout)')
        elif not res.success and 'already' not in res.status_message.lower():
            self.get_logger().warn(f'spawn {name} failed: {res.status_message}')

    def _move_door(self, name, progress):
        x, y, z = self._door_pose(name, progress)
        state = EntityState()
        state.name = name
        state.reference_frame = 'world'
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        p.orientation.w = 1.0
        state.pose = p
        req = SetEntityState.Request()
        req.state = state
        self.set_cli.call_async(req)

    def _tick(self):
        # self-healing robot-pose poll
        if self._robot_future is None:
            req = GetEntityState.Request()
            req.name = self.robot_model
            req.reference_frame = 'world'
            self._robot_future = self.get_cli.call_async(req)
        elif self._robot_future.done():
            res = self._robot_future.result()
            if res is not None and res.success:
                self.robot_xy = (res.state.pose.position.x, res.state.pose.position.y)
            self._robot_future = None

        if self.robot_xy is None:
            self._ticks_without_robot += 1
            if self._ticks_without_robot == int(self.rate * 5):
                self.get_logger().warn(
                    f"can't read pose of model '{self.robot_model}' - no door "
                    f"will open until it is found.")
            return
        self._ticks_without_robot = 0

        rx, ry = self.robot_xy
        for name, d in self.doors.items():
            cx, cy = d['closed'][0], d['closed'][1]
            dist = math.hypot(rx - cx, ry - cy)
            if dist < self.open_dist:
                self.target[name] = 1.0
            elif dist > self.close_dist:
                self.target[name] = 0.0

            slide_len = math.sqrt(sum(v * v for v in d['slide'])) or 1.0
            step = self.slide_speed * self.dt / slide_len
            prog = self.progress[name]
            tgt = self.target[name]
            if prog < tgt:
                prog = min(tgt, prog + step)
            elif prog > tgt:
                prog = max(tgt, prog - step)
            if prog != self.progress[name]:
                self.progress[name] = prog
                self._move_door(name, prog)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDoors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
