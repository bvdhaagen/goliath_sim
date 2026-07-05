#!/usr/bin/env python3
"""
Switchroom status lights.

Puts one emissive light on the DISPLAY above each switch and colours it GREEN
when the switch is ON, RED when OFF. It reads the switch lever/button angles from
the switch_wall joint-state topic (published by a gazebo joint_state_publisher
plugin) and draws each light by spawning a small coloured box via the gazebo
/spawn_entity + /delete_entity services (Gazebo Classic can't recolour a live
visual from ROS without a plugin).

  lever  - latch by angle, once the lever has reached an end AND stopped moving.
  button - act on a fresh press (rising edge) of the ON or OFF slider.

Manual overrides (std_msgs/Bool): /power_on (all) and /<switch>/power (one).
"""
import os

import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


LIGHT_SDF = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <pose>{x} {y} {z} 0 0 0</pose>
    <link name="link"><visual name="v">
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material>
        <ambient>{r} {g} 0 1</ambient>
        <diffuse>{r} {g} 0 1</diffuse>
        <emissive>{er} {eg} 0 1</emissive>
      </material>
    </visual></link>
  </model>
</sdf>"""


class StatusLights(Node):
    def __init__(self):
        super().__init__('status_lights')

        self.declare_parameter('config', '')
        cfg_path = self.get_parameter('config').value
        if not cfg_path or not os.path.exists(cfg_path):
            raise RuntimeError(f'lights config not found: {cfg_path!r}')
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.size = cfg.get('size', [0.14, 0.04, 0.11])
        self.wall_pose = cfg.get('wall_pose', [0.0, 0.0, 0.0])
        self.stop_velocity = float(cfg.get('stop_velocity', 0.05))
        self.switches = cfg['switches']
        self.lights = {n: s['light'] for n, s in self.switches.items()}
        self.state = {n: bool(cfg.get('start_on', False)) for n in self.switches}

        # joint name -> which switch it drives (and role)
        self._lever_of = {}
        self._btn_on_of = {}
        self._btn_off_of = {}
        for name, s in self.switches.items():
            if s.get('type') == 'lever':
                self._lever_of[s['joint']] = name
            elif s.get('type') == 'button':
                self._btn_on_of[s['on_joint']] = name
                self._btn_off_of[s['off_joint']] = name
        self._btn_pressed = {}   # button joint -> was pressed last cycle

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.get_logger().info('Waiting for Gazebo spawn/delete services...')
        self.spawn_cli.wait_for_service()
        self.delete_cli.wait_for_service()

        for name in self.switches:
            self._spawn(name, self.state[name])

        self.create_subscription(Bool, '/power_on', self._on_all, 10)
        for name in self.switches:
            self.create_subscription(
                Bool, f'/{name}/power',
                lambda msg, n=name: self._set(n, msg.data), 10)
        topic = cfg.get('joint_topic', '/switch_wall/joint_states')
        self.create_subscription(JointState, topic, self._on_joints, 10)

        self.get_logger().info(
            f'Status lights ready for {list(self.switches)} (watching {topic}).')

    def _on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

        # Levers: latch by angle, only once stopped.
        for jname, name in self._lever_of.items():
            if jname not in pos:
                continue
            if abs(vel.get(jname, 0.0)) > self.stop_velocity:
                continue
            s = self.switches[name]
            a = abs(pos[jname])
            if a > s['on_above']:
                self._set(name, True)
            elif a < s['off_below']:
                self._set(name, False)

        # Buttons: act only on a fresh press (rising edge), with release hysteresis.
        for jname, name, want_on in (
                [(j, n, True) for j, n in self._btn_on_of.items()] +
                [(j, n, False) for j, n in self._btn_off_of.items()]):
            if jname not in pos:
                continue
            press = self.switches[name]['press']
            a = abs(pos[jname])
            was = self._btn_pressed.get(jname, False)
            now = was
            if a > press:
                now = True
            elif a < 0.5 * press:
                now = False
            if now and not was:
                self._set(name, want_on)
            self._btn_pressed[jname] = now

    # --------------------------------------------------------------- helpers
    def _model_name(self, name, on):
        return f"{name}_light_{'green' if on else 'red'}"

    def _light_xml(self, name, on):
        lx, ly, lz = self.lights[name]
        x = self.wall_pose[0] + lx
        y = self.wall_pose[1] + ly
        z = self.wall_pose[2] + lz
        sx, sy, sz = self.size
        return LIGHT_SDF.format(
            name=self._model_name(name, on), x=x, y=y, z=z, sx=sx, sy=sy, sz=sz,
            r=(0.0 if on else 0.9), g=(0.9 if on else 0.0),
            er=(0.0 if on else 0.5), eg=(0.5 if on else 0.0))

    def _spawn(self, name, on):
        req = SpawnEntity.Request()
        req.name = self._model_name(name, on)
        req.xml = self._light_xml(name, on)
        self.spawn_cli.call_async(req)

    def _delete_model(self, model_name):
        req = DeleteEntity.Request()
        req.name = model_name
        self.delete_cli.call_async(req)

    def _set(self, name, on):
        on = bool(on)
        if self.state.get(name) == on:
            return
        self.state[name] = on
        self._spawn(name, on)                             # add wanted colour
        self._delete_model(self._model_name(name, not on))   # remove other
        self.get_logger().info(f'{name}: {"ON (green)" if on else "OFF (red)"}')

    def _on_all(self, msg):
        for name in self.switches:
            self._set(name, msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = StatusLights()
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
