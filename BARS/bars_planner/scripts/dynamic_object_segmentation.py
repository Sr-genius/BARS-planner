#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dynamic-object segmentation from a world-frame LiDAR point cloud.

A stable background voxel model is built from structure observed across
several consecutive frames. Points that do not belong to a confirmed
background voxel are treated as foreground, clustered by Euclidean region
growing, and retained only when a cluster matches the geometry of a standing
person. The accepted points are published as a dynamic-object cloud; an empty
cloud is published when no object is present, so the planner always has a
valid input.
"""

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class DynamicObjectSegmentation:
    def __init__(self):
        # Topics
        cloud_topic = rospy.get_param("~cloud_topic", "pointcloud")
        self.object_topic = rospy.get_param("~dynamic_object_topic",
                                            "dynamic_objects")
        self.frame_id = rospy.get_param("~frame_id", "world")

        # Background model
        self.voxel = float(rospy.get_param("~voxel", 0.15))
        self.bg_hits = int(rospy.get_param("~bg_hits", 5))
        self.warmup_frames = int(rospy.get_param("~warmup_frames", 10))

        # Clustering
        self.eps = float(rospy.get_param("~eps", 0.35))

        # Geometry filter
        self.min_points = int(rospy.get_param("~min_points", 12))
        self.object_min_h = float(rospy.get_param("~object_min_height", 0.9))
        self.object_max_h = float(rospy.get_param("~object_max_height", 2.0))
        self.max_width = float(rospy.get_param("~max_width", 0.6))
        self.ground_z = float(rospy.get_param("~ground_z", 0.0))
        self.max_ground_z = float(rospy.get_param("~max_ground_z", 0.5))
        self.min_z = float(rospy.get_param("~min_z", 0.05))
        self.max_z = float(rospy.get_param("~max_z", 2.2))

        self.bg_count = {}   # observation count per background voxel key
        self.bg_set = set()  # confirmed background voxel keys
        self.frame_idx = 0

        self.pub = rospy.Publisher(self.object_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(cloud_topic, PointCloud2, self.cloud_cb,
                                    queue_size=1)
        rospy.loginfo("[dynamic_seg] waiting for %s ...", cloud_topic)

    @staticmethod
    def _voxel_keys(pts, voxel):
        idx = np.floor(pts / voxel).astype(np.int64)
        return idx, [tuple(v) for v in idx]

    @staticmethod
    def _cluster_foreground(pts, eps):
        """Greedy Euclidean region growing using a spatial voxel hash."""
        n = len(pts)
        labels = -np.ones(n, dtype=np.int32)
        grid = {}
        vox_idx = np.floor(pts / eps).astype(np.int64)
        for i in range(n):
            grid.setdefault(tuple(vox_idx[i]), []).append(i)

        neighbours = [(a, b, c) for a in (-1, 0, 1)
                      for b in (-1, 0, 1) for c in (-1, 0, 1)]
        cluster_id = 0
        eps2 = eps * eps
        for seed in range(n):
            if labels[seed] != -1:
                continue
            labels[seed] = cluster_id
            stack = [seed]
            while stack:
                cur = stack.pop()
                cx, cy, cz = vox_idx[cur]
                for ox, oy, oz in neighbours:
                    key = (cx + ox, cy + oy, cz + oz)
                    for j in grid.get(key, ()):
                        if labels[j] != -1:
                            continue
                        d = pts[j] - pts[cur]
                        if d.dot(d) <= eps2:
                            labels[j] = cluster_id
                            stack.append(j)
            cluster_id += 1
        return labels, cluster_id

    def cloud_cb(self, msg):
        self.frame_idx += 1
        raw = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float64)
        if pts.shape[0] == 0:
            self._publish(np.zeros((0, 3)))
            return

        # Restrict to the working height band.
        keep = (pts[:, 2] >= self.min_z) & (pts[:, 2] <= self.max_z)
        work = pts[keep]
        if work.shape[0] == 0:
            self._publish(np.zeros((0, 3)))
            return

        vox_idx, keys = self._voxel_keys(work, self.voxel)

        # Update the background model.
        for k in keys:
            count = self.bg_count.get(k, 0) + 1
            self.bg_count[k] = count
            if count >= self.bg_hits:
                self.bg_set.add(k)

        # During warm-up only learn the background and emit nothing.
        if self.frame_idx <= self.warmup_frames:
            self._publish(np.zeros((0, 3)))
            return

        # Foreground extraction, with neighbourhood tolerance.
        is_foreground = np.ones(len(work), dtype=bool)
        for i in range(len(work)):
            ix, iy, iz = vox_idx[i]
            for ox, oy, oz in ((a, b, c) for a in (-1, 0, 1)
                               for b in (-1, 0, 1) for c in (-1, 0, 1)):
                if (ix + ox, iy + oy, iz + oz) in self.bg_set:
                    is_foreground[i] = False
                    break
        foreground = work[is_foreground]
        if len(foreground) < self.min_points:
            self._publish(np.zeros((0, 3)))
            return

        labels, n_clusters = self._cluster_foreground(foreground, self.eps)

        # Geometry filter.
        accepted = []
        for cid in range(n_clusters):
            cluster = foreground[labels == cid]
            if len(cluster) < self.min_points:
                continue
            zmin, zmax = cluster[:, 2].min(), cluster[:, 2].max()
            height = zmax - zmin
            if height < self.object_min_h or height > self.object_max_h:
                continue
            width = max(cluster[:, 0].ptp(), cluster[:, 1].ptp())
            if width > self.max_width:
                continue
            if zmin > self.ground_z + self.max_ground_z:
                continue
            accepted.append(cluster)

        if accepted:
            out = np.concatenate(accepted, axis=0)
            self._publish(out)
            rospy.loginfo_throttle(
                2.0, "[dynamic_seg] %d cluster(s), %d points",
                len(accepted), len(out))
        else:
            self._publish(np.zeros((0, 3)))

    def _publish(self, pts):
        cloud = point_cloud2.create_cloud_xyz32(
            rospy.Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
            pts.astype(np.float32))
        self.pub.publish(cloud)


def main():
    rospy.init_node("dynamic_object_segmentation")
    DynamicObjectSegmentation()
    rospy.spin()


if __name__ == "__main__":
    main()
