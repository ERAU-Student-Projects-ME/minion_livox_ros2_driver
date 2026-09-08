#!/usr/bin/env python3
"""
time_offset_monitor.py

Monitors the timestamp offset between a LiDAR topic (e.g. /livox/lidar)
and a GPS Odometry topic (nav_msgs/Odometry), using
message_filters.ApproximateTimeSynchronizer to pair up messages from
each stream that fall within a small time window of each other.

The GPS's published tf and its Odometry message share the same stamp,
so comparing against Odometry is equivalent to comparing against the
tf, but this runs as a normal topic-to-topic sync instead of a tf
buffer lookup on a timer.

Publishes the per-pair offset (seconds) on /time_offset_monitor/offset_sec
and prints rolling statistics (mean, stddev, min, max) periodically.

A positive offset means the LiDAR timestamp is AHEAD of the GPS
odometry timestamp; negative means it's behind.

Parameters:
  lidar_topic       (string)  default: /livox/lidar
  lidar_msg_type    (string)  default: sensor_msgs/msg/PointCloud2
  gps_topic         (string)  default: /odom
  gps_msg_type      (string)  default: nav_msgs/msg/Odometry
  lidar_qos_reliability  (string)  default: reliable     ["best_effort" | "reliable"]
  lidar_qos_durability   (string)  default: volatile     ["volatile" | "transient_local"]
  lidar_qos_depth        (int)     default: 10
  gps_qos_reliability    (string)  default: best_effort  ["best_effort" | "reliable"]
  gps_qos_durability     (string)  default: volatile     ["volatile" | "transient_local"]
  gps_qos_depth          (int)     default: 10
  sync_slop_sec     (float)   default: 0.05   (max allowed timestamp gap for
                                                message_filters to consider two
                                                messages a matching pair)
  window_size       (int)     default: 200    (samples kept for stats)
  print_period_sec  (float)   default: 2.0

Example usage:
  ros2 run <your_pkg> time_offset_monitor.py --ros-args \
      -p lidar_topic:=/livox/lidar_1HDDH3200106291 \
      -p lidar_qos_reliability:=reliable \
      -p gps_topic:=/gps/odometry \
      -p gps_qos_reliability:=best_effort \
      -p sync_slop_sec:=0.05

Tip: check what QoS your publishers actually use before running, with:
  ros2 topic info <lidar_topic> --verbose
  ros2 topic info <gps_topic> --verbose
and set lidar_qos_*/gps_qos_* to match each -- a mismatch on either one
means that subscription silently never connects. The two topics can
(and often do) use different reliability policies, so each needs its
own QoS, not a shared one.
"""

import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from rosidl_runtime_py.utilities import get_message
from livox_interfaces.msg import CustomMsg

import message_filters


def build_qos(reliability: str, depth: int, durability: str) -> QoSProfile:
    rel = (ReliabilityPolicy.BEST_EFFORT if reliability.lower() == 'best_effort'
           else ReliabilityPolicy.RELIABLE)
    dur = (DurabilityPolicy.TRANSIENT_LOCAL if durability.lower() == 'transient_local'
           else DurabilityPolicy.VOLATILE)
    return QoSProfile(
        reliability=rel,
        durability=dur,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class TimeOffsetMonitor(Node):
    def __init__(self):
        super().__init__('time_offset_monitor')

        self.declare_parameter('lidar_topic', '/livox/lidar')
        self.declare_parameter('lidar_msg_type', 'livox_interfaces/msg/CustomMsg')
        self.declare_parameter('gps_topic', '/gps/odometry')
        self.declare_parameter('gps_msg_type', 'nav_msgs/msg/Odometry')
        self.declare_parameter('lidar_qos_reliability', 'best_effort')
        self.declare_parameter('lidar_qos_durability', 'volatile')
        self.declare_parameter('lidar_qos_depth', 5)
        self.declare_parameter('gps_qos_reliability', 'best_effort')
        self.declare_parameter('gps_qos_durability', 'volatile')
        self.declare_parameter('gps_qos_depth', 5)
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('window_size', 200)
        self.declare_parameter('print_period_sec', 2.0)

        lidar_topic = self.get_parameter('lidar_topic').value
        lidar_type_str = self.get_parameter('lidar_msg_type').value
        gps_topic = self.get_parameter('gps_topic').value
        gps_type_str = self.get_parameter('gps_msg_type').value
        lidar_qos_reliability = self.get_parameter('lidar_qos_reliability').value
        lidar_qos_durability = self.get_parameter('lidar_qos_durability').value
        lidar_qos_depth = self.get_parameter('lidar_qos_depth').value
        gps_qos_reliability = self.get_parameter('gps_qos_reliability').value
        gps_qos_durability = self.get_parameter('gps_qos_durability').value
        gps_qos_depth = self.get_parameter('gps_qos_depth').value
        sync_slop = self.get_parameter('sync_slop_sec').value
        window_size = self.get_parameter('window_size').value
        print_period = self.get_parameter('print_period_sec').value

        self.offsets = deque(maxlen=window_size)

        lidar_qos = build_qos(lidar_qos_reliability, lidar_qos_depth, lidar_qos_durability)
        gps_qos = build_qos(gps_qos_reliability, gps_qos_depth, gps_qos_durability)
        self.get_logger().info(
            f'LiDAR QoS: reliability={lidar_qos_reliability}, '
            f'durability={lidar_qos_durability}, depth={lidar_qos_depth}')
        self.get_logger().info(
            f'GPS QoS: reliability={gps_qos_reliability}, '
            f'durability={gps_qos_durability}, depth={gps_qos_depth}')

        lidar_msg_class = get_message(lidar_type_str)
        gps_msg_class = get_message(gps_type_str)

        # message_filters pairs messages by matching timestamps within
        # sync_slop seconds, avoiding the race of comparing whichever
        # value happened to arrive last on each topic independently.
        # Each subscriber uses its own QoS since the two publishers
        # don't necessarily agree on reliability/durability/depth.
        lidar_sub = message_filters.Subscriber(
            self, lidar_msg_class, lidar_topic, qos_profile=lidar_qos)
        gps_sub = message_filters.Subscriber(
            self, gps_msg_class, gps_topic, qos_profile=gps_qos)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [lidar_sub, gps_sub], queue_size=window_size, slop=sync_slop)
        self.ts.registerCallback(self._synced_cb)

        self.get_logger().info(
            f'Synchronizing LiDAR topic "{lidar_topic}" with '
            f'GPS Odometry topic "{gps_topic}" (slop={sync_slop}s)')

        self.offset_pub = self.create_publisher(
            Float64, 'time_offset_monitor/offset_sec', 10)

        self.create_timer(print_period, self._print_stats)

    def _synced_cb(self, lidar_msg, gps_msg):
        # Called only when message_filters has paired a lidar + gps message
        # within sync_slop_sec of each other -- offset is directly meaningful.
        lidar_stamp = stamp_to_sec(lidar_msg.header.stamp)
        gps_stamp = stamp_to_sec(gps_msg.header.stamp)
        offset = lidar_stamp - gps_stamp
        self.offsets.append(offset)
        out = Float64()
        out.data = offset
        self.offset_pub.publish(out)

    def _print_stats(self):
        if not self.offsets:
            self.get_logger().warn(
                'No synced pairs yet -- check both topics are publishing '
                'and that QoS/slop settings allow a match.')
            return
        vals = list(self.offsets)
        mean = statistics.mean(vals)
        stdev = statistics.pstdev(vals) if len(vals) > 1 else 0.0
        self.get_logger().info(
            f'offset(lidar-gps): mean={mean*1e3:+.3f} ms  '
            f'stdev={stdev*1e3:.3f} ms  '
            f'min={min(vals)*1e3:+.3f} ms  max={max(vals)*1e3:+.3f} ms  '
            f'n={len(vals)}')


def main():
    rclpy.init()
    node = TimeOffsetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
