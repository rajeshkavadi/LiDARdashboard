"""Binary protocols shared by the backend, the SLAM exporter and the DEM service.

Live LiDAR frames keep the original ``LVP1`` layout (see frame_hub.py), which is
deliberately not touched.

Global map protocol (backend -> browser, SLAM exporter -> backend)::

    magic "LVM1" | u8 type | 3 pad | payload

    type 1 TILE_ADD
        int32 tx, int32 ty, int32 tz, uint32 count,
        count * (float32 x, y, z, intensity)      world coordinates

    type 2 TRAJECTORY_APPEND
        uint32 start_index, uint32 count, count * (float64 x, y, z)

    type 3 POSE
        float64 stamp, x, y, z, qx, qy, qz, qw   (map -> body, local frame)

    type 4 RESET
        empty (clears map, trajectory and pose on the viewer/exporter)

DEM protocol (DEM service -> backend -> browser)::

    magic "LVD1" | u8 type | 3 pad | payload

    type 1 PATCH
        float64 origin_x, origin_y
        float32 resolution
        uint32 width, height
        uint32 x0, y0, w, h
        float32 nodata
        w*h float32 heights (NaN = no data)

    type 2 FULL
        same payload, x0 = y0 = 0, w = width, h = height

    type 3 RESET
        empty

All coordinates are local metres.  No geographic CRS is involved.
"""

import struct

MAP_MAGIC = b"LVM1"
DEM_MAGIC = b"LVD1"
ENVELOPE = struct.Struct("<4sB3x")

MAP_TILE_ADD = 1
MAP_TRAJECTORY = 2
MAP_POSE = 3
MAP_RESET = 4

DEM_PATCH = 1
DEM_FULL = 2
DEM_RESET = 3

MAP_TILE_HEADER = struct.Struct("<iiiI")       # tx, ty, tz, count
MAP_TRAJ_HEADER = struct.Struct("<II")         # start index, count
MAP_POSE_STRUCT = struct.Struct("<8d")         # stamp + xyz + quaternion
DEM_HEADER = struct.Struct("<dd f IIIIII f")   # fixed part of PATCH/FULL
POINT_BYTES = 16
TRAJ_POINT_BYTES = 24


def envelope(magic, msg_type):
    return ENVELOPE.pack(magic, msg_type)


def map_tile(tx, ty, tz, points_bytes):
    """Encode one incremental map tile message.

    ``points_bytes`` is a packed sequence of float32 x, y, z, intensity.
    """
    count = len(points_bytes) // POINT_BYTES
    return (envelope(MAP_MAGIC, MAP_TILE_ADD) +
            MAP_TILE_HEADER.pack(tx, ty, tz, count) + points_bytes)


def map_trajectory(start_index, xyz_bytes):
    count = len(xyz_bytes) // TRAJ_POINT_BYTES
    return (envelope(MAP_MAGIC, MAP_TRAJECTORY) +
            MAP_TRAJ_HEADER.pack(start_index, count) + xyz_bytes)


def map_pose(stamp, x, y, z, qx, qy, qz, qw):
    return envelope(MAP_MAGIC, MAP_POSE) + MAP_POSE_STRUCT.pack(
        float(stamp), float(x), float(y), float(z),
        float(qx), float(qy), float(qz), float(qw))


def map_reset():
    return envelope(MAP_MAGIC, MAP_RESET)


def dem_header(origin_x, origin_y, resolution, width, height,
               x0, y0, w, h, nodata):
    return DEM_HEADER.pack(float(origin_x), float(origin_y),
                           float(resolution), int(width), int(height),
                           int(x0), int(y0), int(w), int(h), float(nodata))


def dem_patch(origin_x, origin_y, resolution, width, height,
              x0, y0, w, h, nodata, heights_bytes, full=False):
    return (envelope(DEM_MAGIC, DEM_FULL if full else DEM_PATCH) +
            dem_header(origin_x, origin_y, resolution, width, height,
                       x0, y0, w, h, nodata) + heights_bytes)


def dem_reset():
    return envelope(DEM_MAGIC, DEM_RESET)


def parse_map_message(data):
    """Return ``(type, payload_memoryview)`` or ``(None, None)``."""
    if len(data) < ENVELOPE.size:
        return None, None
    magic, msg_type = ENVELOPE.unpack_from(data, 0)
    if magic != MAP_MAGIC:
        return None, None
    return msg_type, memoryview(data)[ENVELOPE.size:]


def parse_tile(payload):
    if len(payload) < MAP_TILE_HEADER.size:
        return None
    tx, ty, tz, count = MAP_TILE_HEADER.unpack_from(payload, 0)
    body = payload[MAP_TILE_HEADER.size:]
    if len(body) < count * POINT_BYTES:
        return None
    return (tx, ty, tz, count, bytes(body[:count * POINT_BYTES]))


def parse_trajectory(payload):
    if len(payload) < MAP_TRAJ_HEADER.size:
        return None
    start, count = MAP_TRAJ_HEADER.unpack_from(payload, 0)
    body = payload[MAP_TRAJ_HEADER.size:]
    if len(body) < count * TRAJ_POINT_BYTES:
        return None
    return (start, count, bytes(body[:count * TRAJ_POINT_BYTES]))


def parse_pose(payload):
    if len(payload) < MAP_POSE_STRUCT.size:
        return None
    return MAP_POSE_STRUCT.unpack_from(payload, 0)


def parse_dem_message(data):
    if len(data) < ENVELOPE.size:
        return None, None, None
    magic, msg_type = ENVELOPE.unpack_from(data, 0)
    if magic != DEM_MAGIC:
        return None, None, None
    payload = memoryview(data)[ENVELOPE.size:]
    if msg_type == DEM_RESET:
        return msg_type, None, None
    if len(payload) < DEM_HEADER.size:
        return None, None, None
    fields = DEM_HEADER.unpack_from(payload, 0)
    origin_x, origin_y, res, width, height, x0, y0, w, h, nodata = fields
    body = payload[DEM_HEADER.size:]
    need = int(w) * int(h) * 4
    if w < 0 or h < 0 or len(body) < need:
        return None, None, None
    meta = {
        "origin_x": origin_x,
        "origin_y": origin_y,
        "resolution": res,
        "width": int(width),
        "height": int(height),
        "x0": int(x0),
        "y0": int(y0),
        "w": int(w),
        "h": int(h),
        "nodata": nodata,
    }
    return msg_type, meta, bytes(body[:need])
