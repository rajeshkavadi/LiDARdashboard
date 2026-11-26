# -*- coding: utf-8 -*-
"""Backend-agnostic LiDAR odometry core.

This module is the *portable* SLAM fallback of this project.  It needs nothing
beyond NumPy and SciPy (both available from apt on Ubuntu 18.04), no compiled
SLAM package, no network access and no Livox SDK build.  It is used by
``lidar_odometry.py`` (a ROS Melodic / Python 2 node) and by
``tools/odometry_eval.py`` (offline bag evaluation).

What it does
------------
* optional motion compensation (deskew) using the IMU gyro and the real
  per-point timestamps of the Livox PointCloud2 message,
* constant-velocity pose prediction,
* voxel downsampling (bounded memory),
* scan-to-map point-to-point ICP with SVD (Umeyama) pose estimation,
* a bounded local voxel map that is cropped around the sensor.

The output pose and the registered (map-frame) cloud are the *same interface*
FAST-LIO2 exposes (`nav_msgs/Odometry` + `sensor_msgs/PointCloud2`), so the rest
of the pipeline does not care which SLAM produced them.

This is classic ICP-based LiDAR odometry, not a new SLAM algorithm; it has no
loop closure and will drift.  For long surveys FAST-LIO2 / Point-LIO / LIO-SAM
remain the recommended backends and are selected automatically when their
topics are present (see ``slam_web_bridge.py``).
"""

import numpy as np
from scipy.spatial import cKDTree


def voxel_downsample(points, voxel, intensity=None):
    """Return one point per occupied voxel (centroid-free, first point kept).

    ``points`` is an (N, 3) float array.  Kept points are the arithmetic mean of
    the points falling into each voxel, which is slightly smoother than keeping
    the first point and cheap enough at these sizes.
    """
    if points.shape[0] == 0 or voxel <= 0:
        return points, intensity
    keys = np.floor(points / float(voxel)).astype(np.int64)
    # Pack the three integer keys into one for np.unique (much faster than axis=0
    # unique on old NumPy, and available on both Python 2 and 3).
    keys = keys - keys.min(axis=0)
    span = keys.max(axis=0) + 1
    packed = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, index, inverse = np.unique(packed, return_index=True,
                                  return_inverse=True)
    order = np.argsort(index)
    index = index[order]
    # Remap inverse to the sorted unique ordering.
    remap = np.empty_like(order)
    remap[order] = np.arange(order.size)
    inverse = remap[inverse]
    counts = np.bincount(inverse, minlength=index.size).astype(np.float64)
    out = np.zeros((index.size, 3), dtype=np.float64)
    for axis in range(3):
        out[:, axis] = np.bincount(inverse, weights=points[:, axis],
                                   minlength=index.size) / counts
    if intensity is None:
        return out, None
    mean_intensity = np.bincount(inverse, weights=intensity,
                                 minlength=index.size) / counts
    return out, mean_intensity


def best_fit_transform(src, dst):
    """Umeyama/SVD rigid transform mapping ``src`` onto ``dst``."""
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centred = src - src_mean
    dst_centred = dst - dst_mean
    covariance = src_centred.T.dot(dst_centred) / float(src.shape[0])
    u_mat, _, vt_mat = np.linalg.svd(covariance)
    rotation = vt_mat.T.dot(u_mat.T)
    if np.linalg.det(rotation) < 0:
        vt_mat[2, :] *= -1.0
        rotation = vt_mat.T.dot(u_mat.T)
    translation = dst_mean - rotation.dot(src_mean)
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def transform_points(points, transform):
    return points.dot(transform[:3, :3].T) + transform[:3, 3]


def quaternion_from_matrix(transform):
    """Return (x, y, z, w) for the rotation part of ``transform``."""
    m = transform[:3, :3]
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return (x, y, z, w)


def rotation_from_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3)
    axis = axis / norm
    skew = np.array([[0.0, -axis[2], axis[1]],
                     [axis[2], 0.0, -axis[0]],
                     [-axis[1], axis[0], 0.0]])
    return (np.eye(3) + np.sin(angle) * skew +
            (1.0 - np.cos(angle)) * skew.dot(skew))


class LocalVoxelMap(object):
    """Bounded voxel map used as the ICP target."""

    def __init__(self, voxel_size=0.4, max_points=250000, crop_radius=90.0):
        self.voxel_size = float(voxel_size)
        self.max_points = int(max_points)
        self.crop_radius = float(crop_radius)
        self._points = np.zeros((0, 3), dtype=np.float64)
        self._intensity = np.zeros((0,), dtype=np.float64)
        self._occupied = set()
        self._tree = None
        self._dirty = 0

    def __len__(self):
        return self._points.shape[0]

    @property
    def points(self):
        return self._points

    def tree(self):
        if self._tree is None or self._dirty > 0:
            self._tree = cKDTree(self._points) if self._points.shape[0] else None
            self._dirty = 0
        return self._tree

    def _keys(self, points):
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        return keys

    def add(self, points, intensity=None, centre=None):
        if points.shape[0] == 0:
            return 0
        down, down_intensity = voxel_downsample(points, self.voxel_size,
                                                intensity)
        keys = self._keys(down)
        new_rows = []
        new_intensity = []
        for i in range(keys.shape[0]):
            key = (int(keys[i, 0]), int(keys[i, 1]), int(keys[i, 2]))
            if key in self._occupied:
                continue
            self._occupied.add(key)
            new_rows.append(down[i])
            new_intensity.append(down_intensity[i] if down_intensity is not None
                                 else 0.0)
        if not new_rows:
            return 0
        self._points = np.vstack([self._points, np.asarray(new_rows)]).astype(
            np.float64)
        self._intensity = np.concatenate(
            [self._intensity, np.asarray(new_intensity, dtype=np.float64)])
        self._dirty += 1
        if self._points.shape[0] > self.max_points:
            self._crop(centre)
        return len(new_rows)

    def _crop(self, centre):
        if centre is None:
            centre = self._points.mean(axis=0)
        delta = self._points - np.asarray(centre, dtype=np.float64)
        keep = np.sum(delta * delta, axis=1) <= self.crop_radius ** 2
        if keep.all():
            # Radius is not the limiting factor: drop the oldest half instead.
            keep = np.zeros(self._points.shape[0], dtype=bool)
            keep[-self.max_points // 2:] = True
        self._points = self._points[keep]
        self._intensity = self._intensity[keep]
        self._occupied = set(
            (int(k[0]), int(k[1]), int(k[2]))
            for k in self._keys(self._points))
        self._dirty += 1

    def reset(self):
        self._points = np.zeros((0, 3), dtype=np.float64)
        self._intensity = np.zeros((0,), dtype=np.float64)
        self._occupied = set()
        self._tree = None
        self._dirty = 0


class IcpOdometry(object):
    """Scan-to-map ICP odometry in a local (map) frame.

    Parameters
    ----------
    voxel_size : float
        Downsampling resolution for ICP source and map (metres).
    map_voxel : float
        Voxel size of the persistent local map.  Small values give a denser,
        more accurate map but grow memory; the map is cropped around the pose.
    max_iterations, max_correspondence : ICP limits.
    min_range, max_range : sensor range gate.
    """

    def __init__(self, voxel_size=0.35, map_voxel=0.5, max_iterations=12,
                 max_correspondence=1.0, min_range=0.5, max_range=90.0,
                 max_map_points=250000, crop_radius=90.0, min_points=120,
                 deskew=True, process_stride=0):
        self.voxel_size = float(voxel_size)
        self.max_iterations = int(max_iterations)
        self.max_correspondence = float(max_correspondence)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.min_points = int(min_points)
        self.deskew = bool(deskew)
        # 0 = pick a stride per frame so dense sensors stay real time.
        self.process_stride = max(0, int(process_stride))
        # Dense sensors (150k+ points/frame) are decimated to about this many
        # points before ICP, which is what keeps the node real time.
        self.target_points_per_frame = 12000
        self.map = LocalVoxelMap(map_voxel, max_map_points, crop_radius)

        self.pose = np.eye(4)
        self.velocity = np.eye(4)     # body-frame motion between frames
        self.frames = 0
        self.rejected = 0
        self.last_inlier_ratio = 0.0
        self.trajectory = []

    # -- helpers --------------------------------------------------------
    def reset(self):
        self.map.reset()
        self.pose = np.eye(4)
        self.velocity = np.eye(4)
        self.frames = 0
        self.rejected = 0
        self.trajectory = []
        self.last_inlier_ratio = 0.0

    def _prepare(self, points, intensity):
        points = np.asarray(points, dtype=np.float64)
        if points.shape[0] == 0:
            return points, None
        finite = np.isfinite(points).all(axis=1)
        if not finite.all():
            points = points[finite]
            if intensity is not None:
                intensity = intensity[finite]
        ranges = np.sqrt(np.sum(points * points, axis=1))
        keep = (ranges >= self.min_range) & (ranges <= self.max_range)
        points = points[keep]
        if intensity is not None:
            intensity = intensity[keep]
        stride = self.process_stride
        if stride <= 0:   # auto: keep roughly target_points_per_frame
            stride = max(1, int(points.shape[0] // self.target_points_per_frame))
        if stride > 1:
            points = points[::stride]
            if intensity is not None:
                intensity = intensity[::stride]
        return points, intensity

    def _icp(self, source, init):
        tree = self.map.tree()
        if tree is None:
            return init.copy(), 0.0
        transform = init.copy()
        inlier_ratio = 0.0
        for _ in range(self.max_iterations):
            moved = transform_points(source, transform)
            distances, indices = tree.query(
                moved, k=1, distance_upper_bound=self.max_correspondence)
            valid = np.isfinite(distances)
            matched = int(valid.sum())
            inlier_ratio = float(matched) / float(source.shape[0])
            if matched < self.min_points:
                return init.copy(), inlier_ratio
            target = self.map.points[indices[valid]]
            correction = best_fit_transform(moved[valid], target)
            transform = correction.dot(transform)
            step = np.linalg.norm(correction[:3, 3])
            angle = np.arccos(min(1.0, max(-1.0,
                                           (np.trace(correction[:3, :3]) - 1.0)
                                           / 2.0)))
            if step < 1e-4 and angle < 1e-4:
                break
        return transform, inlier_ratio

    # -- main entry -----------------------------------------------------
    def update(self, points, intensity=None, frame_rotation=None):
        """Process one scan.

        ``points`` is (N, 3) in the sensor frame.  ``frame_rotation`` is an
        optional (3, 3) rotation measured by the IMU between the previous and
        the current scan; it is only used as a prediction, never as the result.
        Returns ``(pose, registered_points, registered_intensity, inlier_ratio)``.
        """
        points, intensity = self._prepare(points, intensity)
        if points.shape[0] < 20:
            return self.pose.copy(), None, None, 0.0
        source, source_intensity = voxel_downsample(points, self.voxel_size,
                                                    intensity)

        self.frames += 1
        if len(self.map) == 0:
            self.pose = np.eye(4)
            self.map.add(transform_points(source, self.pose), source_intensity,
                         centre=self.pose[:3, 3])
            self.trajectory.append(self.pose.copy())
            return (self.pose.copy(),
                    transform_points(source, self.pose),
                    source_intensity, 1.0)

        predicted = self.pose.dot(self.velocity)
        if frame_rotation is not None:
            trial = predicted.copy()
            trial[:3, :3] = frame_rotation.dot(predicted[:3, :3])
            # Blend the IMU rotation with the constant-velocity prediction.
            trial[:3, :3] = 0.5 * trial[:3, :3] + 0.5 * predicted[:3, :3]
            u_mat, _, vt_mat = np.linalg.svd(trial[:3, :3])
            trial[:3, :3] = u_mat.dot(vt_mat)
            if np.linalg.det(trial[:3, :3]) > 0:
                predicted = trial

        pose, inlier_ratio = self._icp(source, predicted)
        self.last_inlier_ratio = inlier_ratio
        moved = transform_points(source, pose)

        # Sanity gate: reject a solution that jumps far or fits almost nothing.
        step = pose[:3, 3] - self.pose[:3, 3]
        if np.linalg.norm(step) > 8.0 or inlier_ratio < 0.15:
            self.rejected += 1
            return self.pose.copy(), None, None, inlier_ratio

        self.velocity = np.linalg.inv(self.pose).dot(pose)
        self.pose = pose
        self.map.add(moved, source_intensity, centre=pose[:3, 3])
        self.trajectory.append(pose.copy())
        return pose, moved, source_intensity, inlier_ratio
