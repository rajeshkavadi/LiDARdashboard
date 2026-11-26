#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Built-in LiDAR odometry node (portable fallback for FAST-LIO2).

Why this exists
---------------
FAST-LIO2 must be cloned and compiled against PCL/Eigen/ikd-Tree, which is slow
and machine-specific.  This node gives the dashboard a working GLOBAL MAP on a
plain Ubuntu 18.04 + ROS Melodic install with only ``python-numpy`` and
``python-scipy``.  It publishes exactly the interface FAST-LIO2 publishes, so
the rest of the pipeline (slam_web_bridge.py, DEM, browser) is unchanged and
you can switch backends freely:

    /livox/lidar -> this node -> /Odometry + /cloud_registered (+ /path, TF)

Backends supported by the pipeline (choose with scripts/start_slam.sh):

    builtin   this node (no compile, no network)
    fastlio2  upstream FAST_LIO (hku-mars) -- any package that publishes
              nav_msgs/Odometry + a registered PointCloud2
    external  Point-LIO, LIO-SAM, DLIO, ... (same topic contract)

Parameters (private):
    ~lidar_topic    PointCloud2 topic        (default: auto-detect)
    ~imu_topic      sensor_msgs/Imu topic    (default: auto-detect)
    ~odom_topic     published odometry       (default /Odometry)
    ~cloud_topic    published registered cloud (default /cloud_registered)
    ~path_topic     published nav_msgs/Path  (default /path)
    ~map_frame      (default map)
    ~base_frame     (default livox_frame)
    ~publish_tf     (default true)
    ~voxel_size, ~map_voxel, ~max_iterations, ~max_correspondence,
    ~min_range, ~max_range, ~deskew, ~process_stride, ~max_map_points

Only genuine geometry is used: poses come from scan-to-map ICP, never from a
synthetic or assumed trajectory.
"""

import collections
import math
import os
import sys
import threading
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_core  # noqa: E402

# Hints only: autodetect() falls back to any PointCloud2 / Imu topic, so an
# unlisted bag still works.
CANDIDATE_LIDAR = (
    "/livox/lidar", "/livox/lidar_custom", "/livox/lidar_raw",
    "/livox/points", "/livox/points_raw", "/livox/lidar_front",
    "/livox/lidar_1", "/mid360/livox/lidar", "/mid360/lidar",
    "/mid360/points", "/avia/livox/lidar", "/horizon/livox/lidar",
    "/ouster/points", "/os_cloud_node/points", "/os1_cloud_node/points",
    "/velodyne_points", "/velodyne/points", "/hesai/points",
    "/rslidar_points", "/rslidar/points", "/lslidar_point_cloud",
    "/points", "/points_raw", "/lidar_points", "/lidar/points",
    "/cloud", "/lidar/cloud", "/scan_cloud", "/sensor_points",
)
CANDIDATE_IMU = ("/livox/imu", "/livox/imu_raw", "/mid360/livox/imu",
                 "/mid360/imu", "/avia/livox/imu", "/ouster/imu",
                 "/imu", "/imu/data", "/imu/data_raw", "/imu_raw",
                 "/imu/raw", "/vn100/imu", "/gps/imu")
LIDAR_HINTS = ("livox", "mid360", "lidar", "velodyne", "ouster", "hesai",
               "rslidar", "lslidar", "scan", "points", "cloud", "velo")
IMU_HINTS = ("imu", "ins", "vector", "vn100")
SELF_TOPICS = ("/cloud_registered", "/Odometry", "/path", "/map",
               "/cloud_registered_body")


def log(message):
    rospy.loginfo("[lidar_odometry] " + message)


def published_topics():
    try:
        return dict(rospy.get_published_topics())
    except Exception:  # noqa: BLE001 - mastership race at startup
        return {}


def autodetect(preferred, candidates, wanted_type, hints=(), exclude=()):
    """Resolve a topic: explicit name, exact candidates, name hints, then type."""
    if preferred:
        return preferred
    available = published_topics()
    for name in candidates:
        if name in available and wanted_type in available[name]:
            return name
    for hint in hints:
        for name in sorted(available):
            if name in exclude or hint not in name.lower():
                continue
            if wanted_type in available[name]:
                return name
    for name in sorted(available):
        if name not in exclude and wanted_type in available[name]:
            return name
    return None


def topic_candidates(explicit, candidates, wanted_type, hints=(), exclude=(),
                     limit=6):
    """Every topic that could feed us, best guess first.

    Candidate names win, then keyword matches, then any topic of the wanted
    type.  An explicit ``~lidar_topic`` is tried first but not exclusive: a
    stale parameter or a topic that never publishes must not leave the node
    deaf.  All of them are subscribed and the first one that actually delivers
    data wins.
    """
    available = published_topics()
    ordered = [explicit] if explicit else []
    for name in candidates:
        if name in available and wanted_type in available[name]:
            ordered.append(name)
    for hint in hints:
        for name in sorted(available):
            if name in exclude or hint not in name.lower():
                continue
            if wanted_type in available[name] and name not in ordered:
                ordered.append(name)
    for name in sorted(available):
        if (name not in exclude and wanted_type in available[name]
                and name not in ordered):
            ordered.append(name)
    return ordered[:limit]


class LidarOdometry(object):
    def __init__(self):
        self.explicit_lidar = rospy.get_param("~lidar_topic", "")
        self.explicit_imu = rospy.get_param("~imu_topic", "")
        self.lidar_topic = None
        self.imu_topic = autodetect(self.explicit_imu, CANDIDATE_IMU,
                                    "sensor_msgs/Imu", IMU_HINTS)
        self.lidar_subs = {}
        self.rank = {}          # topic -> candidate priority (lower is better)
        self.imu_sub = None
        candidates = topic_candidates(self.explicit_lidar, CANDIDATE_LIDAR,
                                      "sensor_msgs/PointCloud2", LIDAR_HINTS,
                                      SELF_TOPICS)
        if candidates:
            log("LiDAR candidates: %s" % ", ".join(candidates))
        else:
            rospy.logwarn("[lidar_odometry] no PointCloud2 topic yet; "
                          "waiting for one to appear")

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered")
        self.path_topic = rospy.get_param("~path_topic", "/path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "livox_frame")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.core = slam_core.IcpOdometry(
            voxel_size=float(rospy.get_param("~voxel_size", 0.35)),
            map_voxel=float(rospy.get_param("~map_voxel", 0.5)),
            max_iterations=int(rospy.get_param("~max_iterations", 12)),
            max_correspondence=float(
                rospy.get_param("~max_correspondence", 1.0)),
            min_range=float(rospy.get_param("~min_range", 0.5)),
            max_range=float(rospy.get_param("~max_range", 90.0)),
            max_map_points=int(rospy.get_param("~max_map_points", 250000)),
            deskew=bool(rospy.get_param("~deskew", True)),
            # 0 = automatic decimation for dense sensors (every Nth point)
            process_stride=max(0, int(rospy.get_param("~process_stride", 0))))

        self.queue = collections.deque(maxlen=1)
        self.queue_lock = threading.Lock()
        self.work = threading.Event()
        self.stop = threading.Event()

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=20)
        self.cloud_pub = rospy.Publisher(self.cloud_topic, PointCloud2,
                                         queue_size=2)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=2)
        self.tf_broadcaster = (tf.TransformBroadcaster()
                               if self.publish_tf else None)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.imu_lock = threading.Lock()
        self.imu_samples = collections.deque(maxlen=400)
        self.imu_last = 0.0

        if not self.imu_topic:
            rospy.logwarn("[lidar_odometry] no IMU topic found; running "
                          "LiDAR-only odometry (no deskew, no gyro prior)")
        else:
            self.imu_sub = rospy.Subscriber(self.imu_topic, Imu,
                                            self._on_imu, queue_size=50)
        self._subscribe_candidates(candidates)
        log("LiDAR: %s   IMU: %s" % (self.lidar_topic or "(waiting)",
                                     self.imu_topic or "none"))
        log("publishing %s and %s (frames %s -> %s)"
            % (self.odom_topic, self.cloud_topic, self.map_frame,
               self.base_frame))
        self.discovery = rospy.Timer(rospy.Duration(3.0), self._discover)

        self.worker = threading.Thread(target=self._work_loop,
                                       name="odometry-worker")
        self.worker.daemon = True
        self.worker.start()
        self.stats_timer = rospy.Timer(rospy.Duration(10.0), self._stats)

    # -- input ----------------------------------------------------------
    def _subscribe_candidates(self, candidates):
        for topic in candidates:
            if topic not in self.rank:
                self.rank[topic] = len(self.rank)
            if topic in self.lidar_subs:
                continue
            self.lidar_subs[topic] = rospy.Subscriber(
                topic, PointCloud2,
                lambda msg, t=topic: self._on_cloud(msg, t),
                queue_size=1, buff_size=2 ** 24)

    def _discover(self, _event=None):
        """Find a cloud topic, then drop the ones that lost the race."""
        if self.lidar_topic is not None:
            # A bag can carry several LiDARs (Avia/Mid-360/Ouster).  For the
            # first few frames a better-ranked topic may still take over; after
            # that the choice is frozen and the extra subscriptions go away.
            if self.core.frames >= 5 and len(self.lidar_subs) > 1:
                for name, sub in list(self.lidar_subs.items()):
                    if name == self.lidar_topic:
                        continue
                    try:
                        sub.unregister()
                    except Exception:  # noqa: BLE001 - shutdown race
                        pass
                    self.lidar_subs.pop(name, None)
            return
        candidates = topic_candidates(self.explicit_lidar, CANDIDATE_LIDAR,
                                      "sensor_msgs/PointCloud2", LIDAR_HINTS,
                                      SELF_TOPICS)
        if candidates:
            log("subscribing to %s" % ", ".join(candidates))
            self._subscribe_candidates(candidates)
        if self.imu_sub is None and not self.imu_topic:
            self.imu_topic = autodetect("", CANDIDATE_IMU, "sensor_msgs/Imu",
                                        IMU_HINTS)
            if self.imu_topic:
                self.imu_sub = rospy.Subscriber(self.imu_topic, Imu,
                                                self._on_imu, queue_size=50)
                log("IMU: %s" % self.imu_topic)

    def _on_imu(self, msg):
        stamp = msg.header.stamp.to_sec()
        with self.imu_lock:
            self.imu_samples.append(
                (stamp, msg.angular_velocity.x, msg.angular_velocity.y,
                 msg.angular_velocity.z))
            self.imu_last = time.time()

    def _on_cloud(self, msg, topic=None):
        # Lock onto the first topic that actually delivers frames and drop the
        # subscriptions to the others (stale registrations, shadow topics).
        if self.lidar_topic is None:
            self.lidar_topic = topic
            log("locked onto cloud topic %s" % topic)
        elif topic is not None and topic != self.lidar_topic:
            current = self.rank.get(self.lidar_topic, 1 << 30)
            if self.core.frames < 5 and self.rank.get(topic, 1 << 30) < current:
                log("preferring %s over %s" % (topic, self.lidar_topic))
                self.lidar_topic = topic
            else:
                return
        with self.queue_lock:
            self.queue.clear()
            self.queue.append(msg)
        self.work.set()

    def _gyro_between(self, start, end):
        """Mean angular velocity (rad/s) between two stamps, or None."""
        with self.imu_lock:
            samples = [s for s in self.imu_samples if start <= s[0] <= end]
        if len(samples) < 2 or end <= start:
            return None
        total = np.zeros(3)
        weight = 0.0
        for i in range(1, len(samples)):
            dt = samples[i][0] - samples[i - 1][0]
            if dt <= 0 or dt > 0.5:
                continue
            total += np.array(samples[i][1:4]) * dt
            weight += dt
        if weight <= 0:
            return None
        return total / weight

    # -- message helpers -------------------------------------------------
    @staticmethod
    def _dtype_for(msg, names):
        fields = dict((f.name, f) for f in msg.fields)
        offsets = [fields[n].offset for n in names]
        formats = []
        for name in names:
            size = fields[name].count * 4 if fields[name].name != "timestamp" \
                else 8
            formats.append("<f4" if size == 4 else "<f8")
        return np.dtype({"names": names, "formats": formats,
                         "offsets": offsets, "itemsize": msg.point_step})

    def _extract(self, msg):
        fields = dict((f.name, f) for f in msg.fields)
        if not all(n in fields for n in ("x", "y", "z")):
            return None, None, None
        names = ["x", "y", "z"]
        if "intensity" in fields:
            names.append("intensity")
        dtype = self._dtype_for(msg, names)
        count = msg.width * msg.height
        if count <= 0:
            return None, None, None
        cloud = np.frombuffer(msg.data, dtype=dtype, count=count)
        xyz = np.vstack([cloud["x"], cloud["y"], cloud["z"]]).T.astype(
            np.float64)
        intensity = (cloud["intensity"].astype(np.float64)
                     if "intensity" in names else
                     np.zeros(count, dtype=np.float64))
        stamps = None
        if "timestamp" in fields:
            try:
                stamps = np.frombuffer(
                    msg.data, dtype=self._dtype_for(msg, ["timestamp"]),
                    count=count)["timestamp"].astype(np.float64) / 1e9
            except Exception:  # noqa: BLE001 - malformed per-point time
                stamps = None
        return xyz, intensity, stamps

    def _deskew(self, xyz, stamps, start, end, gyro):
        """Rotate every point into the end-of-scan frame using the gyro.

        Only rotation is compensated (translation during 100 ms is negligible
        for a ground vehicle).  Without real per-point timestamps nothing is
        fabricated: the scan is used as-is.
        """
        if not self.core.deskew or stamps is None or gyro is None:
            return xyz
        span = end - start
        if span <= 0 or span > 0.5:
            return xyz
        omega_norm = float(np.linalg.norm(gyro))
        if omega_norm < 1e-4:
            return xyz
        axis = gyro / omega_norm
        dt = np.clip(end - stamps, 0.0, span)
        # Group by quantised dt to avoid thousands of matrix builds.
        quant = np.round(dt * 200.0).astype(np.int64)
        out = xyz.copy()
        for value in np.unique(quant):
            mask = quant == value
            if not mask.any():
                continue
            rotation = slam_core.rotation_from_axis_angle(
                axis, omega_norm * (value / 200.0))
            out[mask] = xyz[mask].dot(rotation.T)
        return out

    # -- processing ------------------------------------------------------
    def _work_loop(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            if not self.work.wait(0.5):
                continue
            self.work.clear()
            with self.queue_lock:
                msg = self.queue.pop() if self.queue else None
            if msg is None:
                continue
            try:
                self._process(msg)
            except Exception as exc:  # noqa: BLE001 - never kill the node
                rospy.logwarn_throttle(2.0, "[lidar_odometry] frame failed: %s"
                                       % exc)

    def _process(self, msg):
        xyz, intensity, stamps = self._extract(msg)
        if xyz is None or xyz.shape[0] < 50:
            return
        end = msg.header.stamp.to_sec()
        start = end - 0.1
        if stamps is not None and np.isfinite(stamps).any():
            start = float(np.nanmin(stamps))
            end_scan = float(np.nanmax(stamps))
            if end_scan - start > 0.001:
                end = end_scan
        gyro = self._gyro_between(max(start, end - 0.2), end)
        points = self._deskew(xyz, stamps, start, end, gyro)

        frame_rotation = None
        if gyro is not None:
            frame_rotation = slam_core.rotation_from_axis_angle(
                gyro / max(1e-9, np.linalg.norm(gyro)),
                np.linalg.norm(gyro) * max(0.0, end - start))

        pose, registered, reg_intensity, ratio = self.core.update(
            points, intensity, frame_rotation)
        if registered is None:
            return

        stamp = msg.header.stamp
        self._publish_odometry(stamp, pose)
        self._publish_cloud(stamp, registered, reg_intensity)

    def _publish_odometry(self, stamp, pose):
        qx, qy, qz, qw = slam_core.quaternion_from_matrix(pose)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(pose[0, 3])
        odom.pose.pose.position.y = float(pose[1, 3])
        odom.pose.pose.position.z = float(pose[2, 3])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                (pose[0, 3], pose[1, 3], pose[2, 3]), (qx, qy, qz, qw),
                stamp, self.base_frame, self.map_frame)

        entry = PoseStamped()
        entry.header = odom.header
        entry.pose = odom.pose.pose
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(entry)
        if len(self.path_msg.poses) > 20000:
            self.path_msg.poses = self.path_msg.poses[-20000:]
        self.path_pub.publish(self.path_msg)

    def _publish_cloud(self, stamp, points, intensity):
        count = points.shape[0]
        data = np.empty((count, 4), dtype="<f4")
        data[:, :3] = points
        if intensity is not None:
            data[:, 3] = np.clip(intensity, 0.0, 255.0)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.height = 1
        msg.width = count
        msg.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * count
        msg.is_dense = True
        msg.data = data.tobytes()
        self.cloud_pub.publish(msg)

    def _stats(self, _event):
        position = self.core.pose[:3, 3]
        log("frames=%d rejected=%d map_voxels=%d pose=(%.2f, %.2f, %.2f)"
            % (self.core.frames, self.core.rejected, len(self.core.map),
               position[0], position[1], position[2]))

    def shutdown(self):
        self.stop.set()
        self.work.set()


def main():
    rospy.init_node("lidar_odometry", anonymous=False)
    node = LidarOdometry()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
