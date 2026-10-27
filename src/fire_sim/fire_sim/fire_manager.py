"""Tracks fire HP and removes the fire model when it reaches 0.

HP decreases while spraying is effective:
  /spray/active is true (refreshed within SPRAY_TIMEOUT), robot-to-fire distance
  within [min_distance, max_distance] and robot heading within +-max_angle_deg of the direction to the fire.
Robot pose is the Gazebo ground truth (simulation-side judge only).
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity
from std_msgs.msg import Bool, Empty, Float32
from tf2_msgs.msg import TFMessage

FIRE_NAME = 'fire'
UPDATE_PERIOD = 0.1  # [s]
SPRAY_TIMEOUT = 0.5  # [s] spraying stops if /spray/active is not refreshed


class FireManager(Node):

    def __init__(self):
        super().__init__('fire_manager')
        world = self.declare_parameter('world_name', 'house').value
        self.robot_name = self.declare_parameter('robot_name', 'fire_robot').value
        self.max_hp = self.declare_parameter('max_hp', 100.0).value
        self.damage_rate = self.declare_parameter('damage_rate', 10.0).value
        self.min_distance = self.declare_parameter('min_distance', 1.0).value
        self.max_distance = self.declare_parameter('max_distance', 2.0).value
        self.max_angle = math.radians(self.declare_parameter('max_angle_deg', 10.0).value)

        self.fire_xy = None
        self.hp = 0.0
        self.robot_pose = None  # (x, y, yaw)
        self.last_spray_time = None
        self.effective = False
        self.removing = False
        self.last_time = None

        self.delete_client = self.create_client(DeleteEntity, f'/world/{world}/remove')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointStamped, '/fire_sim/spawned', self.on_spawned, latched)
        self.create_subscription(TFMessage, '/sim/ground_truth_poses', self.on_poses, 10)
        self.create_subscription(Bool, '/spray/active', self.on_spray, 10)
        self.hp_pub = self.create_publisher(Float32, '/fire_sim/hp', 10)
        self.extinguished_pub = self.create_publisher(Empty, '/fire_sim/extinguished', 10)
        self.create_timer(UPDATE_PERIOD, self.update)

    def on_spawned(self, msg):
        self.fire_xy = (msg.point.x, msg.point.y)
        self.hp = self.max_hp
        self.removing = False
        self.last_time = None
        self.get_logger().info(f'Tracking fire at ({msg.point.x:.2f}, {msg.point.y:.2f}), HP {self.hp:.0f}')

    def on_poses(self, msg):
        for tf in msg.transforms:
            if tf.child_frame_id == self.robot_name:
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.robot_pose = (t.x, t.y, yaw)
                return

    def on_spray(self, msg):
        self.last_spray_time = self.get_clock().now() if msg.data else None

    def check_effective(self, now):
        """Return (effective, reason) for the current spray state."""
        if (self.last_spray_time is None
                or (now - self.last_spray_time).nanoseconds * 1e-9 > SPRAY_TIMEOUT):
            return False, 'not spraying'
        if self.robot_pose is None:
            return False, 'no robot pose'
        rx, ry, yaw = self.robot_pose
        dx, dy = self.fire_xy[0] - rx, self.fire_xy[1] - ry
        dist = math.hypot(dx, dy)
        angle = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
        if not self.min_distance <= dist <= self.max_distance:
            return False, f'distance {dist:.2f} m out of range'
        if abs(angle) > self.max_angle:
            return False, f'angle {math.degrees(angle):.1f} deg out of range'
        return True, f'distance {dist:.2f} m, angle {math.degrees(angle):.1f} deg'

    def update(self):
        if self.fire_xy is None or self.removing:
            return
        now = self.get_clock().now()
        dt = 0.0 if self.last_time is None else (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        effective, reason = self.check_effective(now)
        if effective != self.effective:
            self.effective = effective
            state = 'effective' if effective else 'not effective'
            self.get_logger().info(f'Spray {state}: {reason}')
        if effective:
            self.hp = max(0.0, self.hp - self.damage_rate * dt)
        self.hp_pub.publish(Float32(data=self.hp))

        if self.hp <= 0.0:
            self.remove_fire()

    def remove_fire(self):
        if not self.delete_client.service_is_ready():
            self.get_logger().warn('Delete service not ready')
            return
        self.removing = True
        req = DeleteEntity.Request()
        req.entity.name = FIRE_NAME
        req.entity.type = Entity.MODEL
        self.delete_client.call_async(req).add_done_callback(self.on_removed)

    def on_removed(self, future):
        if not future.result() or not future.result().success:
            self.get_logger().error('Fire removal failed, retrying')
            self.removing = False
            return
        self.get_logger().info('Fire extinguished')
        self.fire_xy = None
        self.effective = False
        self.extinguished_pub.publish(Empty())


def main():
    rclpy.init()
    try:
        rclpy.spin(FireManager())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
