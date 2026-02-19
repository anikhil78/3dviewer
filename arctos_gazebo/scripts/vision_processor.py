#!/usr/bin/env python3
"""
vision_processor.py — Colour-based 3D object detection from an RGBD camera.

Subscribes to the /vision/* topics published by vision_sim.launch.py,
detects red / blue / green objects, back-projects their centroids into 3D
using the depth image and camera intrinsics, then publishes labelled
RViz markers and logs the world-frame pose of each object.

This script is designed to be the starting point for the real vision pipeline:
swap the /vision/* topic prefix for whatever your physical camera publishes and
the same code runs on real hardware.

Usage
-----
  # Terminal 1 — start the vision sim
  ros2 launch arctos_gazebo vision_sim.launch.py

  # Terminal 2 — run this script
  ros2 run arctos_gazebo vision_processor

  # Optional flags
  ros2 run arctos_gazebo vision_processor -- --debug   # show OpenCV windows
  ros2 run arctos_gazebo vision_processor -- --topic-prefix /vision

Outputs
-------
  /vision/detected_objects   visualization_msgs/MarkerArray  (RViz spheres)
  /vision/debug_image        sensor_msgs/Image               (HSV masks overlay)

Extending
---------
  - Replace colour thresholding with a neural-net detector by overriding
    the _detect() method.
  - Attach the detected pose to a MoveIt pick goal by importing
    move_arm_moveit.py and calling mover.move_to_pose().
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import cv2
from cv_bridge import CvBridge

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  — registers PointStamped transform
from geometry_msgs.msg import PointStamped


# ------------------------------------------------------------------ #
# Colour definitions                                                   #
# HSV ranges for OpenCV (H: 0–179, S/V: 0–255).                      #
# Each entry: (label, bgr_for_display, lower1, upper1, lower2, upper2)#
# Two ranges for red because it wraps around H=0/179.                 #
# ------------------------------------------------------------------ #
COLOURS = [
    {
        'name':    'red',
        'display': (0, 0, 220),          # BGR for OpenCV drawing
        'ranges':  [
            (np.array([0,   120, 60]), np.array([10,  255, 255])),
            (np.array([165, 120, 60]), np.array([179, 255, 255])),
        ],
        'marker_rgba': (0.9, 0.1, 0.1, 0.85),
    },
    {
        'name':    'blue',
        'display': (220, 80, 0),
        'ranges':  [
            (np.array([100, 120, 60]), np.array([130, 255, 255])),
        ],
        'marker_rgba': (0.1, 0.1, 0.9, 0.85),
    },
    {
        'name':    'green',
        'display': (0, 200, 0),
        'ranges':  [
            (np.array([45, 100, 60]), np.array([80, 255, 255])),
        ],
        'marker_rgba': (0.1, 0.85, 0.1, 0.85),
    },
]

# Minimum blob area in pixels to count as a valid detection.
MIN_BLOB_PX = 200


class VisionProcessor(Node):
    def __init__(self, topic_prefix: str, debug: bool):
        super().__init__('vision_processor')
        self._bridge = CvBridge()
        self._debug  = debug
        self._prefix = topic_prefix.rstrip('/')

        # Camera intrinsics — populated on first CameraInfo message.
        self._fx = self._fy = self._cx = self._cy = None

        # ---------------------------------------------------------- #
        # TF2 buffer for camera_optical_frame → world transforms     #
        # ---------------------------------------------------------- #
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------------------------------------------------- #
        # Subscribers                                                  #
        # Synchronise colour image and depth image by timestamp.      #
        # ---------------------------------------------------------- #
        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._info_sub = self.create_subscription(
            CameraInfo,
            f'{self._prefix}/camera_info',
            self._camera_info_cb,
            sensor_qos,
        )

        img_sub   = message_filters.Subscriber(
            self, Image, f'{self._prefix}/image',
            qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(
            self, Image, f'{self._prefix}/depth_image',
            qos_profile=sensor_qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub], queue_size=5, slop=0.05)
        self._sync.registerCallback(self._image_cb)

        # ---------------------------------------------------------- #
        # Publishers                                                   #
        # ---------------------------------------------------------- #
        self._marker_pub = self.create_publisher(
            MarkerArray, f'{self._prefix}/detected_objects', 10)
        self._debug_pub  = self.create_publisher(
            Image, f'{self._prefix}/debug_image', sensor_qos)

        self.get_logger().info(
            f'VisionProcessor ready — prefix={self._prefix}, debug={debug}')
        self.get_logger().info(
            f'  Subscribing to {self._prefix}/image + {self._prefix}/depth_image')
        self.get_logger().info(
            f'  Publishing to  {self._prefix}/detected_objects')

    # ---------------------------------------------------------------- #
    # Camera intrinsics callback                                         #
    # ---------------------------------------------------------------- #
    def _camera_info_cb(self, msg: CameraInfo):
        if self._fx is not None:
            return  # already set — camera_info is static
        K = msg.k   # row-major 3×3 intrinsics matrix
        self._fx, self._fy = K[0], K[4]
        self._cx, self._cy = K[2], K[5]
        self._camera_frame = msg.header.frame_id
        self.get_logger().info(
            f'Camera intrinsics: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}  frame={self._camera_frame}')

    # ---------------------------------------------------------------- #
    # Synchronised image + depth callback                               #
    # ---------------------------------------------------------------- #
    def _image_cb(self, img_msg: Image, depth_msg: Image):
        if self._fx is None:
            self.get_logger().warn('Waiting for camera_info…', throttle_duration_sec=2.0)
            return

        # Convert to OpenCV arrays
        bgr   = self._bridge.imgmsg_to_cv2(img_msg,   'bgr8')
        depth = self._bridge.imgmsg_to_cv2(depth_msg, '32FC1')  # metres, NaN=invalid

        detections = self._detect(bgr, depth, img_msg.header)

        self._publish_markers(detections, img_msg.header)

        if self._debug or True:  # always publish debug image for RViz
            debug_img = self._draw_debug(bgr, detections)
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(debug_img, 'bgr8'))

        if self._debug:
            cv2.imshow('VisionProcessor — detections', debug_img)
            cv2.waitKey(1)

    # ---------------------------------------------------------------- #
    # Detection: colour threshold → centroid → 3D back-projection      #
    # ---------------------------------------------------------------- #
    def _detect(self, bgr, depth, header) -> list:
        """
        For each colour, find the largest blob in the masked image, compute
        its 2D centroid, sample depth there, back-project to 3D in the
        camera optical frame, then transform to the world frame via TF2.

        Returns a list of dicts:
          { name, u, v, depth_m, x_cam, y_cam, z_cam,
            world_x, world_y, world_z, colour }
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        results = []

        for colour in COLOURS:
            # Build combined mask from all HSV ranges for this colour
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in colour['ranges']:
                mask |= cv2.inRange(hsv, lo, hi)

            # Morphological cleanup
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours, keep the largest
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area < MIN_BLOB_PX:
                continue

            # 2D centroid via image moments
            M = cv2.moments(largest)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])

            # Depth at centroid — average over a 5×5 patch for robustness
            h, w = depth.shape
            v0, v1 = max(0, v-2), min(h, v+3)
            u0, u1 = max(0, u-2), min(w, u+3)
            patch = depth[v0:v1, u0:u1]
            valid = patch[np.isfinite(patch) & (patch > 0.05)]
            if valid.size == 0:
                continue
            d = float(np.median(valid))

            # Back-project to camera optical frame
            x_cam = (u - self._cx) * d / self._fx
            y_cam = (v - self._cy) * d / self._fy
            z_cam = d

            # Transform to world frame
            pt_cam = PointStamped()
            pt_cam.header = header
            pt_cam.header.frame_id = self._camera_frame
            pt_cam.point.x = x_cam
            pt_cam.point.y = y_cam
            pt_cam.point.z = z_cam

            try:
                pt_world = self._tf_buffer.transform(
                    pt_cam, 'world', timeout=rclpy.duration.Duration(seconds=0.1))
                wx, wy, wz = (pt_world.point.x,
                              pt_world.point.y,
                              pt_world.point.z)
            except Exception as e:
                self.get_logger().warn(
                    f'TF {self._camera_frame}→world failed: {e}',
                    throttle_duration_sec=2.0)
                wx = wy = wz = float('nan')

            results.append({
                'name':    colour['name'],
                'u': u, 'v': v,
                'depth_m': d,
                'x_cam': x_cam, 'y_cam': y_cam, 'z_cam': z_cam,
                'world_x': wx,  'world_y': wy,  'world_z': wz,
                'colour':  colour,
                'contour': largest,
                'area':    area,
            })

            self.get_logger().info(
                f'[{colour["name"]:6s}] pixel=({u},{v})  depth={d:.3f}m  '
                f'world=({wx:.3f}, {wy:.3f}, {wz:.3f}) m',
                throttle_duration_sec=0.5,
            )

        return results

    # ---------------------------------------------------------------- #
    # Publish RViz markers                                              #
    # ---------------------------------------------------------------- #
    def _publish_markers(self, detections: list, header):
        array = MarkerArray()

        # Delete all old markers first
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        array.markers.append(del_marker)

        for i, det in enumerate(detections):
            if not np.isfinite(det['world_x']):
                continue

            r, g, b, a = det['colour']['marker_rgba']

            # Sphere at the detected 3D world position
            sphere = Marker()
            sphere.header.frame_id = 'world'
            sphere.header.stamp    = header.stamp
            sphere.ns              = 'detected_objects'
            sphere.id              = i
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position.x = det['world_x']
            sphere.pose.position.y = det['world_y']
            sphere.pose.position.z = det['world_z']
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
            sphere.color.r = r
            sphere.color.g = g
            sphere.color.b = b
            sphere.color.a = a
            sphere.lifetime.sec = 1  # auto-expire if no new detection

            # Text label above the sphere
            label = Marker()
            label.header        = sphere.header
            label.ns            = 'labels'
            label.id            = i + 100
            label.type          = Marker.TEXT_VIEW_FACING
            label.action        = Marker.ADD
            label.pose.position.x = det['world_x']
            label.pose.position.y = det['world_y']
            label.pose.position.z = det['world_z'] + 0.08
            label.pose.orientation.w = 1.0
            label.scale.z       = 0.04
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text          = (f"{det['name']}\n"
                                   f"({det['world_x']:.3f}, "
                                   f"{det['world_y']:.3f}, "
                                   f"{det['world_z']:.3f}) m")
            label.lifetime.sec  = 1

            array.markers.extend([sphere, label])

        self._marker_pub.publish(array)

    # ---------------------------------------------------------------- #
    # Debug image overlay                                               #
    # ---------------------------------------------------------------- #
    def _draw_debug(self, bgr, detections) -> np.ndarray:
        out = bgr.copy()
        for det in detections:
            c = det['colour']['display']
            cv2.drawContours(out, [det['contour']], -1, c, 2)
            cv2.circle(out, (det['u'], det['v']), 6, c, -1)
            cv2.putText(
                out,
                f"{det['name']} {det['depth_m']:.2f}m",
                (det['u'] + 8, det['v']),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2,
            )
        return out


# ------------------------------------------------------------------ #
# Entry point                                                          #
# ------------------------------------------------------------------ #
def main():
    parser = argparse.ArgumentParser(
        description='RGBD colour-based 3D object detector.')
    parser.add_argument(
        '--debug', action='store_true',
        help='Show live OpenCV windows (requires a display)')
    parser.add_argument(
        '--topic-prefix', default='/vision', metavar='PREFIX',
        help='ROS topic namespace prefix (default: /vision)')
    # ros2 run passes args after "--"
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = VisionProcessor(topic_prefix=args.topic_prefix, debug=args.debug)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if args.debug:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
