"""Nozzle driver: shows the powder spray in Gazebo while /spray/active is true.

Spraying stops if /spray/active is not refreshed within SPRAY_TIMEOUT.
The spray is a particle-emitter model spawned at the nozzle and removed when spraying
stops. Toggling an emitter on the robot is not used: that command lives for a single
simulation step and the Gazebo GUI misses it under load, while model creation and removal
always reach the GUI. The robot stands still while spraying, so a static model is enough.

This is a simulation-side visual effect, so the nozzle pose comes from the robot's
ground-truth pose (bridged by fire_robot_gazebo/sim.launch.py). Spawn/remove calls go
one at a time and are retried until Gazebo confirms them.
"""
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

SPRAY_TIMEOUT = 0.5  # [s]
MAX_ATTEMPTS = 5
SPRAY_MODEL = 'spray'


class SprayNozzle(Node):

    def __init__(self):
        super().__init__('spray_nozzle')
        world = self.declare_parameter('world_name', 'house').value
        self.robot_name = self.declare_parameter('robot_name', 'fire_robot').value
        # Nozzle pose in the robot model frame (see fire_robot.urdf.xacro)
        self.nozzle_x = self.declare_parameter('nozzle_x', 0.17).value
        self.nozzle_z = self.declare_parameter('nozzle_z', 0.11).value
        self.nozzle_pitch = self.declare_parameter('nozzle_pitch', -0.08).value
        with open(os.path.join(get_package_share_directory('fire_mission'),
                               'models', 'spray.sdf')) as f:
            self.spray_sdf = f.read()

        self.last_on = None
        self.wanted = False    # spray requested
        self.shown = False     # spray model confirmed in Gazebo
        self.attempts = 0
        self.busy = False      # a spawn/remove call is in flight
        self.changed_at = None
        self.robot_pose = None  # (x, y, z, yaw)
        self.spawn_client = self.create_client(SpawnEntity, f'/world/{world}/create')
        self.remove_client = self.create_client(DeleteEntity, f'/world/{world}/remove')
        self.create_subscription(Bool, '/spray/active', self.on_spray, 10)
        self.create_subscription(TFMessage, '/sim/ground_truth_poses', self.on_poses, 10)
        self.create_timer(0.1, self.update)

    def on_spray(self, msg):
        self.last_on = self.get_clock().now() if msg.data else None

    def on_poses(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.robot_name:
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.robot_pose = (t.x, t.y, t.z, yaw)
                return

    def update(self):
        on = (self.last_on is not None
              and (self.get_clock().now() - self.last_on).nanoseconds * 1e-9 <= SPRAY_TIMEOUT)
        if on != self.wanted:
            self.wanted = on
            self.attempts = 0
            self.changed_at = self.get_clock().now()
            self.get_logger().info('Nozzle ON' if on else 'Nozzle OFF')

        if self.busy or self.wanted == self.shown:
            return
        if self.attempts >= MAX_ATTEMPTS:
            if self.attempts == MAX_ATTEMPTS:
                self.get_logger().error(
                    f'Could not {"show" if self.wanted else "hide"} the spray in Gazebo')
                self.attempts += 1
            return
        client = self.spawn_client if self.wanted else self.remove_client
        if not client.service_is_ready() or (self.wanted and self.robot_pose is None):
            return
        self.attempts += 1
        self.busy = True
        target = self.wanted
        future = client.call_async(self.spawn_request() if target else self.remove_request())
        future.add_done_callback(lambda fut: self.on_done(fut, target))

    def on_done(self, future, target):
        self.busy = False
        result = future.result()
        if result is not None and result.success:
            self.shown = target
            dt = (self.get_clock().now() - self.changed_at).nanoseconds * 1e-9
            self.get_logger().info(f'Spray {"shown" if target else "removed"} ({dt:.1f} s)')

    def spawn_request(self):
        x, y, z, yaw = self.robot_pose
        cp, sp = math.cos(self.nozzle_pitch / 2.0), math.sin(self.nozzle_pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        req = SpawnEntity.Request()
        req.entity_factory.name = SPRAY_MODEL
        req.entity_factory.sdf = self.spray_sdf
        pose = req.entity_factory.pose
        pose.position.x = x + self.nozzle_x * math.cos(yaw)
        pose.position.y = y + self.nozzle_x * math.sin(yaw)
        pose.position.z = z + self.nozzle_z
        # yaw (z) then pitch (y)
        pose.orientation.x = -sp * sy
        pose.orientation.y = sp * cy
        pose.orientation.z = cp * sy
        pose.orientation.w = cp * cy
        return req

    def remove_request(self):
        req = DeleteEntity.Request()
        req.entity.name = SPRAY_MODEL
        req.entity.type = Entity.MODEL
        return req


def main():
    rclpy.init()
    try:
        rclpy.spin(SprayNozzle())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
