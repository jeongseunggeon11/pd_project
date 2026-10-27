"""Detects fire in the thermal camera image (L16, 0.01 K per unit).

A frame is a hit when at least min_pixels exceed threshold_k. Fire is confirmed after
confirm_frames consecutive hits and cleared after clear_frames consecutive misses.
The bearing (rad, + = left) is taken from the horizontal centroid of the hot pixels.
"""
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32


class FireDetector(Node):

    def __init__(self):
        super().__init__('fire_detector')
        self.threshold_k = self.declare_parameter('threshold_k', 450.0).value
        self.resolution_k = self.declare_parameter('resolution_k', 0.01).value
        self.min_pixels = self.declare_parameter('min_pixels', 4).value
        self.confirm_frames = self.declare_parameter('confirm_frames', 3).value
        self.clear_frames = self.declare_parameter('clear_frames', 3).value
        self.hfov = self.declare_parameter('hfov', 1.0472).value

        self.raw_threshold = int(self.threshold_k / self.resolution_k)
        self.hits = 0
        self.misses = 0
        self.confirmed = False

        self.create_subscription(Image, '/thermal/image', self.on_image, 1)
        self.detected_pub = self.create_publisher(Bool, '/fire_detection/detected', 10)
        self.bearing_pub = self.create_publisher(Float32, '/fire_detection/bearing', 10)

    def on_image(self, msg):
        if msg.encoding != 'mono16':
            self.get_logger().error(f'Unexpected encoding {msg.encoding}', throttle_duration_sec=5.0)
            return
        dtype = '>u2' if msg.is_bigendian else '<u2'
        img = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
        cols = np.nonzero(img >= self.raw_threshold)[1]
        hit = cols.size >= self.min_pixels

        if hit:
            self.hits += 1
            self.misses = 0
        else:
            self.misses += 1
            self.hits = 0

        if not self.confirmed and self.hits >= self.confirm_frames:
            self.confirmed = True
        elif self.confirmed and self.misses >= self.clear_frames:
            self.confirmed = False
            self.get_logger().info('Fire lost')

        self.detected_pub.publish(Bool(data=self.confirmed))
        if self.confirmed and hit:
            fx = (msg.width / 2.0) / math.tan(self.hfov / 2.0)
            cx = cols.mean() + 0.5
            bearing = math.atan((msg.width / 2.0 - cx) / fx)
            self.bearing_pub.publish(Float32(data=bearing))
            if self.hits == self.confirm_frames:
                self.get_logger().info(
                    f'Fire confirmed: bearing {math.degrees(bearing):.1f} deg, {cols.size} px')


def main():
    rclpy.init()
    try:
        rclpy.spin(FireDetector())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
