#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Convert Livox sensor_msgs/PointCloud2 to livox_ros_driver/CustomMsg.

FAST-LIO2's Livox path expects CustomMsg with a per-point ``offset_time``.
The bag produced by the existing driver publishes PointCloud2 with x/y/z,
intensity, tag, line and an absolute per-point ``timestamp`` (float64, ns).
This node uses those real timestamps; it refuses to publish if the field is
missing rather than inventing motion-compensation timing.

Run under ROS Melodic (Python 2.7). It is a read-only adapter: the original
topic, driver and RViz configuration are untouched.

Parameters (private):
    ~input_topic   default /livox/lidar
    ~output_topic  default /livox/lidar_custom
    ~frame_override default "" (keep the input frame)
"""

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg, CustomPoint

REQUIRED = ("x", "y", "z", "timestamp")


class LivoxPc2ToCustom(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        self.output_topic = rospy.get_param("~output_topic", "/livox/lidar_custom")
        self.frame_override = rospy.get_param("~frame_override", "")
        self.frame_count = 0
        self.warned = False

        self.publisher = rospy.Publisher(
            self.output_topic, CustomMsg, queue_size=2)
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud2, self._on_cloud,
            queue_size=2, buff_size=2 ** 24)
        rospy.loginfo("[livox_pc2_to_custom] %s -> %s" %
                      (self.input_topic, self.output_topic))

    def _on_cloud(self, msg):
        fields = dict((f.name, f) for f in msg.fields)
        missing = [name for name in REQUIRED if name not in fields]
        if missing:
            rospy.logerr_throttle(
                5.0,
                "[livox_pc2_to_custom] refusing to publish: missing %s. "
                "FAST-LIO2 needs real per-point timestamps; record with the "
                "Livox driver's CustomMsg or a PointCloud2 that carries them."
                % ", ".join(missing))
            return

        count = msg.width * msg.height
        if count <= 0:
            return

        names = [n for n in ("x", "y", "z", "intensity", "tag", "line",
                             "timestamp") if n in fields]
        formats = []
        for name in names:
            if name == "timestamp":
                formats.append("<f8" if fields[name].datatype == 8 else "<f4")
            elif name in ("tag", "line"):
                formats.append("<u1")
            else:
                formats.append("<f4")
        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": [fields[n].offset for n in names],
            "itemsize": msg.point_step,
        })
        cloud = np.frombuffer(msg.data, dtype=dtype, count=count)

        header_ns = int(round(msg.header.stamp.to_sec() * 1e9))
        if header_ns <= 0:
            rospy.logerr_throttle(
                5.0, "[livox_pc2_to_custom] header stamp is zero; refusing "
                     "to fabricate a timebase")
            return
        stamps = np.asarray(cloud["timestamp"], dtype=np.float64)
        offsets = np.rint(stamps - header_ns).astype(np.int64)
        offsets = np.clip(offsets, 0, 2 ** 32 - 1)
        span_ns = int(offsets.max()) if count else 0
        if span_ns > 500000000:
            rospy.logwarn_throttle(
                5.0, "[livox_pc2_to_custom] point time span is %.1f ms; "
                     "check whether the timestamp field is absolute ns"
                     % (span_ns / 1e6))

        out = CustomMsg()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = (self.frame_override or msg.header.frame_id)
        out.timebase = np.uint64(header_ns)
        out.point_num = np.uint32(count)
        out.lidar_id = np.uint8(0)
        out.rsvd = [0, 0, 0]

        intensity = cloud["intensity"] if "intensity" in names else np.zeros(count)
        tag = cloud["tag"] if "tag" in names else np.zeros(count, dtype=np.uint8)
        line = cloud["line"] if "line" in names else np.zeros(count, dtype=np.uint8)

        points = [None] * count
        xs = cloud["x"]
        ys = cloud["y"]
        zs = cloud["z"]
        for i in xrange(count):
            point = CustomPoint()
            point.offset_time = np.uint32(offsets[i])
            point.x = float(xs[i])
            point.y = float(ys[i])
            point.z = float(zs[i])
            point.reflectivity = np.uint8(min(255, max(0, int(intensity[i]))))
            point.tag = np.uint8(tag[i])
            point.line = np.uint8(line[i])
            points[i] = point
        out.points = points
        self.publisher.publish(out)

        self.frame_count += 1
        if self.frame_count % 50 == 0:
            rospy.loginfo(
                "[livox_pc2_to_custom] %d frames, %d points, %.1f ms span" %
                (self.frame_count, count, span_ns / 1e6))


def main():
    rospy.init_node("livox_pc2_to_custom", anonymous=False)
    LivoxPc2ToCustom()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
