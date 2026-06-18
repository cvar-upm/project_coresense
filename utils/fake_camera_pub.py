#!/usr/bin/env python3
"""Publish a synthetic (blank) camera image stream for a drone's nadir camera.

Broadcasts a static TF from droneN/gimbal → droneN/camera_link with a
downward-pointing ROS optical frame (Z down, X body-right, Y body-back).
The image header uses camera_link so the overlay projects from directly below.

Usage:
    python3 utils/fake_camera_pub.py --namespace drone0
    python3 utils/fake_camera_pub.py --namespace drone0 --width 1280 --height 720 --fps 15
"""

__authors__ = 'Guillermo GP-Lenza'
__license__ = 'BSD-3-Clause'

import argparse
import sys

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# 180° rotation about the (1,−1,0)/√2 axis:
#   X_cam = −Y_gimbal  (body-right  → image-right)
#   Y_cam = −X_gimbal  (body-back   → image-top points drone-forward)
#   Z_cam = −Z_gimbal  (body-down   → optical axis points at ground)
# Required by visionToOgreRotation which converts ROS optical → Ogre convention.
_NADIR_Q = (math.sqrt(2) / 2, -math.sqrt(2) / 2, 0.0, 0.0)  # xyzw

_DEFAULT_WIDTH = 640
_DEFAULT_HEIGHT = 480
_DEFAULT_FPS = 10.0
# Horizontal FOV ~90 degrees: fx = width / (2 * tan(pi/4)) = width / 2
_DEFAULT_FOV_H = 90.0


class FakeCameraPub(Node):
    def __init__(self, namespace: str, width: int, height: int, fps: float,
                 fov_h_deg: float):
        super().__init__('fake_camera_pub', namespace=namespace)
        self._namespace = namespace
        self._width = width
        self._height = height
        self._camera_frame = f'{namespace}/camera_link'

        self._broadcast_nadir_tf(namespace)

        fov_h_rad = math.radians(fov_h_deg)
        fx = (width / 2.0) / math.tan(fov_h_rad / 2.0)
        fov_v_rad = 2.0 * math.atan(math.tan(fov_h_rad / 2.0) * height / width)
        fy = (height / 2.0) / math.tan(fov_v_rad / 2.0)
        cx = width / 2.0
        cy = height / 2.0

        self._camera_info = CameraInfo()
        self._camera_info.width = width
        self._camera_info.height = height
        self._camera_info.distortion_model = 'plumb_bob'
        self._camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._blank = np.full((height, width, 3), 255, dtype=np.uint8)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._img_pub = self.create_publisher(
            Image, 'sensor_measurements/image_raw', sensor_qos)
        self._info_pub = self.create_publisher(
            CameraInfo, 'sensor_measurements/camera_info', sensor_qos)

        period = 1.0 / fps
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f'[{namespace}] nadir camera_link frame published on '
            f'{namespace}/gimbal → {self._camera_frame}'
        )
        self.get_logger().info(
            f'[{namespace}] fake camera {width}x{height} @ {fps} Hz '
            f'| frame: {self._camera_frame} '
            f'| fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}'
        )

    def _broadcast_nadir_tf(self, namespace: str) -> None:
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f'{namespace}/gimbal'
        t.child_frame_id = f'{namespace}/camera_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = _NADIR_Q[0]
        t.transform.rotation.y = _NADIR_Q[1]
        t.transform.rotation.z = _NADIR_Q[2]
        t.transform.rotation.w = _NADIR_Q[3]
        self._static_tf_broadcaster.sendTransform(t)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        hdr = Header(stamp=stamp, frame_id=self._camera_frame)

        img_msg = Image()
        img_msg.header = hdr
        img_msg.height = self._height
        img_msg.width = self._width
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = self._width * 3
        img_msg.data = self._blank.tobytes()
        self._img_pub.publish(img_msg)

        info = self._camera_info
        info.header = hdr
        self._info_pub.publish(info)


def main() -> int:
    parser = argparse.ArgumentParser(description='Synthetic camera publisher for the AR overlay')
    parser.add_argument('--namespace', '-n', default='drone0',
                        help='Drone ROS namespace (default: drone0)')
    parser.add_argument('--width', type=int, default=_DEFAULT_WIDTH)
    parser.add_argument('--height', type=int, default=_DEFAULT_HEIGHT)
    parser.add_argument('--fps', type=float, default=_DEFAULT_FPS)
    parser.add_argument('--fov', type=float, default=_DEFAULT_FOV_H,
                        help='Horizontal field of view in degrees (default: 90)')
    args = parser.parse_args()

    rclpy.init()
    node = FakeCameraPub(args.namespace, args.width, args.height, args.fps, args.fov)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
