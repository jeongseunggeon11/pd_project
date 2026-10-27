"""Spawns a fire model at a random candidate position after a random delay.

The first delay starts on /patrol/started (or immediately if wait_for_patrol is false).
Only one fire exists at a time; the next one is scheduled after /fire_sim/extinguished.
"""
import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import Empty

FIRE_NAME = 'fire'


class FireGenerator(Node):

    def __init__(self):
        super().__init__('fire_generator')
        world = self.declare_parameter('world_name', 'house').value
        self.min_delay = self.declare_parameter('min_delay', 30.0).value
        self.max_delay = self.declare_parameter('max_delay', 120.0).value
        wait_for_patrol = self.declare_parameter('wait_for_patrol', True).value
        # No default: without the params file there are no valid positions
        flat = self.declare_parameter('candidates', Parameter.Type.DOUBLE_ARRAY).value or []
        self.candidates = list(zip(flat[0::2], flat[1::2]))
        if not self.candidates:
            self.get_logger().error('No fire candidates (parameter "candidates" not set), '
                                    'not spawning fires')

        with open(os.path.join(get_package_share_directory('fire_sim'), 'models', 'fire.sdf')) as f:
            self.fire_sdf = f.read()

        self.spawn_client = self.create_client(SpawnEntity, f'/world/{world}/create')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.spawned_pub = self.create_publisher(PointStamped, '/fire_sim/spawned', latched)
        self.create_subscription(Empty, '/fire_sim/extinguished', self.on_extinguished, 10)

        self.timer = None
        self.started = False
        if wait_for_patrol:
            self.create_subscription(Empty, '/patrol/started', self.on_patrol_started, latched)
            self.get_logger().info('Waiting for /patrol/started')
        else:
            self.on_patrol_started(None)

    def on_patrol_started(self, _msg):
        if not self.started:
            self.started = True
            self.schedule_next()

    def schedule_next(self):
        if not self.candidates:
            return
        delay = random.uniform(self.min_delay, self.max_delay)
        self.get_logger().info(f'Next fire in {delay:.1f} s')
        self.timer = self.create_timer(delay, self.spawn_fire)

    def spawn_fire(self):
        self.destroy_timer(self.timer)
        self.timer = None
        if not self.spawn_client.service_is_ready():
            self.get_logger().warn('Spawn service not ready, retrying in 2 s')
            self.timer = self.create_timer(2.0, self.spawn_fire)
            return

        x, y = random.choice(self.candidates)
        req = SpawnEntity.Request()
        req.entity_factory.name = FIRE_NAME
        req.entity_factory.sdf = self.fire_sdf
        req.entity_factory.pose.position.x = x
        req.entity_factory.pose.position.y = y
        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda fut: self.on_spawned(fut, x, y))

    def on_spawned(self, future, x, y):
        if not future.result() or not future.result().success:
            self.get_logger().error('Fire spawn failed, retrying in 2 s')
            self.timer = self.create_timer(2.0, self.spawn_fire)
            return
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = x, y
        self.spawned_pub.publish(msg)
        self.get_logger().info(f'Fire spawned at ({x:.2f}, {y:.2f})')

    def on_extinguished(self, _msg):
        if self.timer is None:
            self.schedule_next()


def main():
    rclpy.init()
    try:
        rclpy.spin(FireGenerator())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
