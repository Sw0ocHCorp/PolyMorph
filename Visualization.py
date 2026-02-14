import ast
from collections import deque
import math
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from math import degrees



# ===== Configuration =====
UDP_IP = "127.0.0.1"
UDP_PORT = 9000

# Expected big-endian chunk IDs
LIDAR_CHUNK_ID       =  0x0005
LIDAR_RANGE_ID       =  0x000A
LIDAR_MEASUREMENTS_ID=  0x000B
LIDAR_OBSTACLES_ID   =  0x000C

# ==== IMU / POSE IDs (from your Rust) ====
POSE_LOCATION_ID         = 0x000D
POSE_ORIENTATION_ID      = 0x000E
POSE_LINEAR_VEL_ID       = 0x000F
POSE_ANGULAR_VEL_ID      = 0x001A

# Optional: top-level IMU/pose chunk id (adapt if needed)
IMU_CHUNK_ID             = 0x0004  # example, change to your real value
DEBUG_CHUNK= 0xFEEF
HISTORY_SIZE= 100


# --- UDP Setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False) # Non-blocking to keep UI responsive

# --- Buffers for Time Series ---
times = deque(maxlen=HISTORY_SIZE)
true_roll_buff = deque(maxlen=HISTORY_SIZE)
true_pitch_buff = deque(maxlen=HISTORY_SIZE)
true_yaw_buff = deque(maxlen=HISTORY_SIZE)
est_roll_buff = deque(maxlen=HISTORY_SIZE)
est_pitch_buff = deque(maxlen=HISTORY_SIZE)
est_yaw_buff = deque(maxlen=HISTORY_SIZE)
frame_counter = 0

def euler_to_rotmat(roll, pitch, yaw):
    """XYZ (roll, pitch, yaw) -> 3x3 rotation matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr,  cr]])

    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])

    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])

    return Rz @ Ry @ Rx  # yaw * pitch * roll


def _parse_vec3_f32_be(data: bytes, idx: int):
    """Parse 3 big-endian f32 values and return (np.array([x,y,z]), new_idx)."""
    vals = struct.unpack_from('>fff', data, idx)
    return np.array(vals, dtype=float), idx + 12


def parse_imu_frame(data: bytes):
    """
    Parse a frame containing IMU pose data.

    Expected layout (example):
      0–1   : SOF = 0xABCD
      2–3   : frame size (u16, big-endian)
      4–5   : chunk id (u16) == IMU_CHUNK_ID
      then a sequence of:
          2 bytes field id + 12 bytes (3×f32) payload, big-endian
      for each of:
          POSE_LOCATION_ID
          POSE_ORIENTATION_ID
          POSE_LINEAR_VEL_ID
          POSE_ANGULAR_VEL_ID
    """
    idx = 0
    true_orientation= None
    # SOF
    if data[idx:idx+2] != b'\xAB\xCD':
        return None
    idx += 2

    # Frame size
    frame_size, = struct.unpack_from('>H', data, idx)
    idx += 2
    # Optionally check consistency
    # if frame_size != len(data): return None

    # Chunk id
    chunk_id, = struct.unpack_from('>H', data, idx)
    idx += 2
    # Handle the specific 0xFEEF Debug Chunk
    if chunk_id == DEBUG_CHUNK:
        idx+=2
        # Based on your description: FRAME | 0xFEEF | orientation (3xf32)
        if idx + 12 <= len(data):
            true_orientation, idx = _parse_vec3_f32_be(data, idx)
    elif chunk_id != IMU_CHUNK_ID:
        return None

    location = orientation = lin_vel = ang_vel = None

    # Loop until buffer end, reading id + vec3
    while idx + 2 <= len(data):
        field_id, = struct.unpack_from('>H', data, idx)
        idx += 2

        # not enough bytes for vec3: stop
        if idx + 12 > len(data):
            break
        if field_id == POSE_LOCATION_ID:
            location, idx = _parse_vec3_f32_be(data, idx)
        elif field_id == POSE_ORIENTATION_ID:
            orientation, idx = _parse_vec3_f32_be(data, idx)
        elif field_id == POSE_LINEAR_VEL_ID:
            lin_vel, idx = _parse_vec3_f32_be(data, idx)
        elif field_id == POSE_ANGULAR_VEL_ID:
            ang_vel, idx = _parse_vec3_f32_be(data, idx)
        else:
            # Unknown field: skip its vec3 payload
            idx += 12

    # Return as dict (you can wrap in a small class if you prefer)
    return {
        "location":        location,
        "orientation":     orientation,
        "linear_velocity": lin_vel,
        "angular_velocity": ang_vel,
        "true_orientation": true_orientation
    }

def parse_lidar_map_frame(data: bytes):
    idx = 0
    # 1. Handle Header (SOF and Frame Size)
    # The Rust 'to_bytes' starts at the Chunk ID, but your Python 
    # receiver loop usually expects the 4-byte ABCD + Size header.
    if data[idx:idx+2] != b'\xAB\xCD':
        return None
    idx += 4  # Skip SOF (2) and Frame Size (2)

    # 2. Chunk ID
    chunk_id, = struct.unpack_from('>H', data, idx)
    idx += 2
    if chunk_id != LIDAR_CHUNK_ID:
        return None

    # 3. Obstacle ID (LIDAR_OBSTACLES)
    idx += 2 
    
    # 4. Total Byte Size of objects
    # In Rust: (self.lidar_objects.len() * 16) as u16
    total_payload_size, = struct.unpack_from('>H', data, idx)
    idx += 2
    
    num_objects = total_payload_size // 16
    objects = []

    # 5. Parse the bounding boxes
    for _ in range(num_objects):
        # Unpack xMin, yMin, xMax, yMax (4 x f32 = 16 bytes)
        bbox = struct.unpack_from('>ffff', data, idx)
        idx += 16
        
        # Convert to a format useful for plotting (e.g., a numpy array)
        # Each 'obj' is (xMin, yMin, xMax, yMax)
        objects.append(np.array(bbox))

    return objects

def parse_lidar_frame(data: bytes):
    idx = 0
    test= len(data)
    try:
        # ===== SOF =====
        if data[idx:idx+2] != b'\xAB\xCD':
            return None, None, None
        idx += 2

        # ===== FRAME SIZE =====
        frame_size, = struct.unpack_from('>H', data, idx)
        idx += 2

        # ===== CHUNK ID =====
        chunk_id, = struct.unpack_from('>H', data, idx)
        idx += 2
        if chunk_id == 4:
            test= 1
        if chunk_id != LIDAR_CHUNK_ID:
            return None, None, None

        # ===== RANGE ID =====
        range_id, = struct.unpack_from('>H', data, idx)
        idx += 2

        # ===== FIRST & LAST ANGLE =====
        first_angle, angle_step = struct.unpack_from('>ff', data, idx)
        idx += 8

        # ===== MEASUREMENTS ID =====
        meas_id, = struct.unpack_from('>H', data, idx)
        idx += 2

        # ===== NUMBER OF MEASUREMENTS =====
        num_meas, = struct.unpack_from('>H', data, idx)
        idx += 2

        # ===== MEASUREMENTS =====
        ranges = np.array(struct.unpack_from(f'>{num_meas}f', data, idx))
        idx += 4*num_meas
        # ===== ANGLE GENERATION =====
        # Convert to radians for plotting/math
        if num_meas == 1:
            angles = np.array([first_angle])
        else:
            angles= np.array([first_angle + math.pi/2 + i * angle_step for i in range(num_meas)])
            #angles = first_angle + np.arange(num_meas) * angle_step
            """angle_step = (last_angle - first_angle) / (num_meas - 1)
            angles = first_angle + np.arange(num_meas) * angle_step"""
        # ===== OBSTACLE ID =====
        obstacles, = struct.unpack_from('>H', data, idx)
        idx += 2
        if obstacles != LIDAR_OBSTACLES_ID: return None, None
        n_obstacles, = struct.unpack_from('>H', data, idx)
        idx += 2
        obstacle_pts = np.array(struct.unpack_from(f'>{n_obstacles}H', data, idx))
        idx += 2*n_obstacles
        return angles, ranges, obstacle_pts
    except Exception as e:
        print(f"Parse error: {e}")
        return None, None

# --- Updated Plotting Setup with Subplots ---
fig = plt.figure(figsize=(12, 12))

# Top: 2D LiDAR
ax1 = fig.add_subplot(3, 1, 1)
# Create an empty collection for the boxes
rect_collection = PatchCollection([], facecolor='red', edgecolor='darkred', alpha=0.6)
ax1.add_collection(rect_collection)

ax1.set_xlim(-15, 15)
ax1.set_ylim(-15, 15)
ax1.set_aspect('equal')
ax1.set_title("Real-time 2D Lidar Bounding Boxes")

# Bottom: 3D orientation
ax2 = fig.add_subplot(3, 1, 2, projection='3d')
# One origin, three direction vectors (x,y,z body axes)
origin = np.array([[0.0, 0.0, 0.0]])
dirs = np.eye(3)  # unit X, Y, Z in body frame

# Create a dummy quiver; will be updated in `update`
body_axes = ax2.quiver(
    origin[:, 0], origin[:, 1], origin[:, 2],
    dirs[:, 0], dirs[:, 1], dirs[:, 2],
    length=0.5, normalize=True, colors=['r', 'g', 'b']
)

ax2.set_title("Robot Orientation (Body Axes)")
ax2.set_xlim(-1, 1)
ax2.set_ylim(-1, 1)
ax2.set_zlim(-1, 1)
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")
ax2.grid(True)

# 3. Time Series Comparison Subplot
ax3 = fig.add_subplot(3, 1, 3)
line_tr, = ax3.plot([], [], 'r-', label='True Roll')
line_tp, = ax3.plot([], [], 'g-', label='True Pitch')
line_ty, = ax3.plot([], [], 'b-', label='True Yaw')
line_er, = ax3.plot([], [], 'r--', alpha=0.5, label='Est Roll')
line_ep, = ax3.plot([], [], 'g--', alpha=0.5, label='Est Pitch')
line_ey, = ax3.plot([], [], 'b--', alpha=0.5, label='Est Yaw')
ax3.set_xlim(0, HISTORY_SIZE)
ax3.set_ylim(-np.pi, np.pi)
ax3.legend(loc='upper right', ncol=2, fontsize='small')
ax3.set_title("Orientation Comparison (Radians)")

# Adjust layout so titles/labels don’t overlap
plt.tight_layout()

def update(frame):
    global body_axes, rect_collection

    try:
        data = None
        while True:
            try:
                data, addr = sock.recvfrom(4096)
            except BlockingIOError:
                break

        if data:
            try:
                # LiDAR
                lidar_bboxes = parse_lidar_map_frame(data)
                if lidar_bboxes is not None:
                    patches = []
                    for bbox in lidar_bboxes:
                        # bbox = [xMin, yMin, xMax, yMax]
                        x_min, y_min, x_max, y_max = bbox
                        
                        width = x_max - x_min
                        height = y_max - y_min
                        
                        # Create the rectangle: Rectangle((lower_left_x, lower_left_y), width, height)
                        rect = Rectangle((x_min, y_min), width, height)
                        patches.append(rect)
                    
                    # Update the collection with the new list of rectangles
                    rect_collection.set_paths(patches)
                """angles, ranges, obstacle_pts = parse_lidar_frame(data)
                if angles is not None:
                    x = ranges * np.cos(angles)
                    y = ranges * np.sin(angles)

                    colors = np.zeros(len(ranges))
                    last_idx = 0
                    for obs_id, end_idx in enumerate(obstacle_pts):
                        end_idx = min(end_idx, len(ranges))
                        colors[last_idx : end_idx + 1] = obs_id + 1
                        last_idx = end_idx + 1

                    scatter.set_offsets(np.column_stack((x, y)))
                    scatter.set_array(colors)
                    if len(obstacle_pts) > 0:
                        scatter.set_clim(0, len(obstacle_pts) + 1)"""
            except:
                pass
            try:
                # IMU / Pose
                imu = parse_imu_frame(data)
                
                if imu is not None:
                    if imu["orientation"] is not None:
                        roll, pitch, yaw = imu["orientation"]  # radians
                        est_roll_buff.append(imu["orientation"][0])
                        est_pitch_buff.append(imu["orientation"][1])
                        est_yaw_buff.append(imu["orientation"][2])

                        R = euler_to_rotmat(roll, pitch, yaw)

                        # Rotate basis vectors
                        dirs_world = R @ np.eye(3)

                        # Remove old quiver (matplotlib 3D API has no clean in-place update)
                        body_axes.remove()
                        body_axes = ax2.quiver(
                            [0, 0, 0], [0, 0, 0], [0, 0, 0],
                            dirs_world[0, :], dirs_world[1, :], dirs_world[2, :],
                            length=0.5, normalize=True, colors=['r', 'g', 'b']
                        )
                    if imu["true_orientation"] is not None: 
                        true_roll_buff.append(imu["true_orientation"][0])
                        true_pitch_buff.append(imu["true_orientation"][1])
                        true_yaw_buff.append(imu["true_orientation"][2])
                if len(est_roll_buff) > 0:
                    x_data = np.arange(len(est_roll_buff))
                    line_er.set_data(x_data, list(est_roll_buff))
                    line_ep.set_data(x_data, list(est_pitch_buff))
                    line_ey.set_data(x_data, list(est_yaw_buff))
                if len(true_roll_buff) > 0:
                    x_data = np.arange(len(true_roll_buff))
                    line_tr.set_data(x_data, list(true_roll_buff))
                    line_tp.set_data(x_data, list(true_pitch_buff))
                    line_ty.set_data(x_data, list(true_yaw_buff))
                
                #ax3.set_xlim(max(0, frame_counter - HISTORY_SIZE), frame_counter)    
            except:
                pass
            try:
                # Convert bytes → string
                string_data = data.decode('utf-8')

                # Convert string → tuple
                numbers = ast.literal_eval(string_data)
                true_roll_buff.append(numbers[0])
                true_pitch_buff.append(numbers[2])
                true_yaw_buff.append(numbers[1])
            except:
                pass
                
    except Exception as e:
        print(f"Update error: {e}")

    return rect_collection, body_axes, line_tr, line_tp, line_ty, line_er, line_ep, line_ey


ani = FuncAnimation(fig, update, interval=20, repeat_delay= 20, blit=False, cache_frame_data=False)
plt.show()