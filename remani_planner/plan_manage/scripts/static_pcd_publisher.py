#!/usr/bin/env python3
"""Publish a static XYZ PCD as PointCloud2 in the PCD's native coordinates.

Optional processing is applied in this order:

1. rigid transform into ``world`` (``~T_world_cloud``, identity by default);
2. optional display-only voxel downsampling (``~display_voxel_leaf_size``).

The transform maps the PCD's own frame onto ``world``; the PCD file itself is
never rewritten.
"""
import math
import struct
import sys

DEFAULT_FRAME = "world"


def _header(path):
    values = {}
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            values[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                return values, stream.tell()


def load_xyz_points(path):
    meta, data_offset = _header(path)
    fields = meta.get("FIELDS", [])
    try:
        indices = [fields.index(name) for name in ("x", "y", "z")]
    except ValueError as exc:
        raise ValueError("PCD must contain x, y and z fields") from exc
    data_type = meta["DATA"][0].lower()
    if data_type == "ascii":
        points = []
        with open(path, "rb") as stream:
            stream.seek(data_offset)
            for line in stream:
                values = line.split()
                if values:
                    points.append(tuple(float(values[i]) for i in indices))
        return points
    if data_type != "binary":
        raise ValueError("only ASCII and binary PCD are supported")
    sizes = [int(v) for v in meta["SIZE"]]
    types = meta["TYPE"]
    counts = [int(v) for v in meta.get("COUNT", ["1"] * len(fields))]
    if any(sizes[i] != 4 or types[i] != "F" or counts[i] != 1 for i in indices):
        raise ValueError("x, y and z must be scalar float32 fields")
    point_step = sum(size * count for size, count in zip(sizes, counts))
    offsets = []
    offset = 0
    for size, count in zip(sizes, counts):
        offsets.append(offset)
        offset += size * count
    count = int(meta.get("POINTS", meta["WIDTH"])[0])
    points = []
    with open(path, "rb") as stream:
        stream.seek(data_offset)
        raw = stream.read(point_step * count)
    for index in range(count):
        base = index * point_step
        points.append(tuple(struct.unpack_from("<f", raw, base + offsets[i])[0]
                            for i in indices))
    return points


def voxel_downsample(points, leaf_size):
    """Return one centroid per occupied voxel, preserving first-seen order."""
    if leaf_size <= 0.0:
        return list(points)
    inverse = 1.0 / leaf_size
    buckets = {}
    order = []
    for point in points:
        key = (int(math.floor(point[0] * inverse)),
               int(math.floor(point[1] * inverse)),
               int(math.floor(point[2] * inverse)))
        bucket = buckets.get(key)
        if bucket is None:
            buckets[key] = [point[0], point[1], point[2], 1]
            order.append(key)
        else:
            bucket[0] += point[0]
            bucket[1] += point[1]
            bucket[2] += point[2]
            bucket[3] += 1
    return [(buckets[key][0] / buckets[key][3],
             buckets[key][1] / buckets[key][3],
             buckets[key][2] / buckets[key][3]) for key in order]


def rotation_matrix(roll, pitch, yaw):
    """Rotation R = Rz(yaw) * Ry(pitch) * Rx(roll), as used by ROS RPY."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def apply_transform(points, translation=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    """Apply the rigid transform T_world_cloud to PCD-native points."""
    if tuple(translation) == (0.0, 0.0, 0.0) and tuple(rpy) == (0.0, 0.0, 0.0):
        return list(points)
    rotation = rotation_matrix(*rpy)
    tx, ty, tz = translation
    return [(rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + tx,
             rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + ty,
             rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + tz)
            for x, y, z in points]


def _vector_param(rospy, name, default):
    value = rospy.get_param(name, default)
    if len(value) != 3:
        raise ValueError("%s must have exactly 3 elements" % name)
    return tuple(float(component) for component in value)


def quaternion_to_yaw(x, y, z, w):
    """Yaw (rotation about world Z) of a quaternion, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def placement_anchor(points):
    """Anchor used by interactive placement: cloud centre in XY, floor in Z."""
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]
    return (0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys)), min(zs))


def place_points(points, anchor, position, yaw):
    """Move the anchored cloud so its anchor lands on position with the given yaw."""
    ax, ay, az = anchor
    px, py, pz = position
    c, s = math.cos(yaw), math.sin(yaw)
    return [(c * (x - ax) - s * (y - ay) + px,
             s * (x - ax) + c * (y - ay) + py,
             z - az + pz) for x, y, z in points]


def main():
    import threading
    import rospy
    from sensor_msgs.msg import PointCloud2, PointField

    rospy.init_node("static_pcd_publisher")
    path = rospy.get_param("~pcd_file")
    frame = rospy.get_param("~frame_id", DEFAULT_FRAME)
    topic = rospy.get_param("~topic", "/map_generator/global_cloud")
    display_topic = rospy.get_param("~display_topic", "")
    rate_hz = rospy.get_param("~rate", 1.0)
    display_leaf_size = float(rospy.get_param("~display_voxel_leaf_size", 0.0))
    pose_topic = rospy.get_param("~pose_topic", "/initialpose")
    translation = _vector_param(rospy, "~T_world_cloud/translation", [0.0, 0.0, 0.0])
    rpy = _vector_param(rospy, "~T_world_cloud/rpy", [0.0, 0.0, 0.0])

    points = load_xyz_points(path)
    raw_count = len(points)
    points = list(points)
    points = apply_transform(points, translation, rpy)
    anchor = placement_anchor(points)

    initial_pose = rospy.get_param("~initial_pose", [])
    if len(initial_pose) == 4:
        initial_position = tuple(float(v) for v in initial_pose[:3])
        initial_yaw = float(initial_pose[3])
        points = place_points(points, anchor, initial_position, initial_yaw)
    else:
        initial_position = anchor
        initial_yaw = rpy[2]


    fields = [PointField("x", 0, PointField.FLOAT32, 1),
              PointField("y", 4, PointField.FLOAT32, 1),
              PointField("z", 8, PointField.FLOAT32, 1)]
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    display_publisher = (rospy.Publisher(display_topic, PointCloud2, queue_size=1, latch=True)
                         if display_topic else None)
    from geometry_msgs.msg import PoseStamped
    pose_publisher = rospy.Publisher("~current_pose", PoseStamped, queue_size=1, latch=True)
    lock = threading.Lock()
    state = {"points": points}

    def publish_pose(position, yaw):
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = rospy.Time.now()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        pose_publisher.publish(message)

    def publish_points(current, target_publisher):
        payload = b"".join(struct.pack("<fff", *point) for point in current)
        message = PointCloud2(height=1, width=len(current), fields=fields,
                              is_bigendian=False, point_step=12,
                              row_step=12 * len(current), data=payload,
                              is_dense=True)
        message.header.frame_id = frame
        message.header.stamp = rospy.Time.now()
        target_publisher.publish(message)

    def publish_all(current):
        publish_points(current, publisher)
        if display_publisher is not None:
            publish_points(voxel_downsample(current, display_leaf_size), display_publisher)

    def on_pose(msg):
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation.x, pose.orientation.y,
                                pose.orientation.z, pose.orientation.w)
        position = (pose.position.x, pose.position.y, pose.position.z)
        placed = place_points(points, anchor, position, yaw)
        with lock:
            state["points"] = placed
        publish_all(placed)
        publish_pose(position, yaw)
        rospy.loginfo("Re-placed cloud at [%.2f %.2f %.2f] yaw=%.2f",
                      position[0], position[1], position[2], yaw)

    if pose_topic:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, on_pose)

    publish_pose(initial_position, initial_yaw)
    rospy.loginfo("Loaded %d points from %s; planner_points=%d; display_topic=%s "
                  "display_voxel_leaf_size=%.3f display_points=%d; "
                  "T_world_cloud translation=%s rpy=%s anchor=%s initial_pose=%s; "
                  "publishing %s in frame %s; interactive pose topic %s",
                  raw_count, path, len(points), display_topic or "<disabled>",
                  display_leaf_size,
                  len(voxel_downsample(points, display_leaf_size)), translation, rpy,
                  anchor, list(initial_position) + [initial_yaw], topic, frame,
                  pose_topic or "<disabled>")
    while not rospy.is_shutdown():
        with lock:
            current = state["points"]
        publish_all(current)
        rospy.sleep(1.0 / max(rate_hz, 0.01))


if __name__ == "__main__":
    try:
        main()
    except (IOError, OSError, ValueError) as error:
        print("static_pcd_publisher: %s" % error, file=sys.stderr)
        sys.exit(2)
