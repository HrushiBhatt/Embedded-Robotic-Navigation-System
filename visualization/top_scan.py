# =============================================================================
#  top_scan.py - Radar view + dead-reckoned global map for the radar-mapping robot
# =============================================================================
#
#  Two live windows:
#    1. Polar radar  - objects from the latest scans (angle, distance, angular width)
#    2. Field map    - every object projected into a global x/y frame using the
#                      robot's dead-reckoned pose, then re-drawn relative to the robot
#
#  Pose tracking
#  -------------
#    heading : updated locally on each turn key (a/d = +/-1 deg, 9/0 = +/-90 deg)
#    x, y    : advanced from the robot's "Distance Moved" telemetry along the heading
#
#  Objects within 5 cm of an existing map point are treated as duplicates.
#
#  Keys: robot commands are forwarded as-is.  'm' clears the radar,
#        'n' clears the field map, 'esc' disconnects.
# =============================================================================

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import keyboard
import threading
import math

# === Roomba Connection Info ===
HOST = '192.168.1.1'
PORT = 288
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# === Radar Data Storage ===
# Parallel lists: index i describes the same detected object across all lists
degree = []        # midpoint angle (rad)
rarr = []          # distance (cm)
sizearr = []       # 'small' | 'medium' | 'large'
min_degrees = []   # start angle (rad)
max_degrees = []   # end angle (rad)
path_angles = []
path_dists = []

# === Field Map Data ===
roomba_heading = 0  # degrees
roomba_x = 0
roomba_y = 0
field_objects = []  # (x, y, size) in the global frame

# === Setup Radar Plot ===
fig = plt.figure()
fig.set_size_inches(6, 6)
ax = fig.add_subplot(projection='polar')

# === Setup Field Map Plot ===
fig2, ax2 = plt.subplots()
fig2.set_size_inches(6, 6)
ax2.set_title("Roomba Global-Centered Map")
ax2.set_xlim(-200, 200)
ax2.set_ylim(-200, 200)
ax2.set_aspect('equal')


# === Helper Functions ===
def get_color(size):
    return {'small': 'green', 'medium': 'orange', 'large': 'red'}.get(size.lower(), 'gray')


# Sensor frame -> global offset.  Scan angle 90 deg = straight ahead, so subtract
# 90 to align it with the robot's heading before rotating into the global frame.
def polar_to_cartesian_global(dist, angle_deg, heading_deg):
    total_angle_rad = math.radians((heading_deg + angle_deg - 90) % 360)
    x = dist * math.cos(total_angle_rad)
    y = dist * math.sin(total_angle_rad)
    return x, y


# Global point -> robot-centred frame (translate by pose, rotate by -heading).
def transform_to_roomba_frame(x, y):
    dx = x - roomba_x
    dy = y - roomba_y
    theta = math.radians(roomba_heading)
    x_rel = dx * math.cos(-theta) - dy * math.sin(-theta)
    y_rel = dx * math.sin(-theta) + dy * math.cos(-theta)
    return x_rel, y_rel


# Redraw the field map with the robot fixed at the origin, facing +x.
def update_field_map():
    ax2.clear()
    ax2.set_title("Roomba Global-Centered Map")
    ax2.set_xlim(-200, 200)
    ax2.set_ylim(-200, 200)
    ax2.set_aspect('equal')
    ax2.plot(0, 0, 'ko', label='Roomba')
    dx = 15 * math.cos(math.radians(0))
    dy = 15 * math.sin(math.radians(0))
    ax2.arrow(0, 0, dx, dy, head_width=5, fc='blue', ec='blue')
    for ox, oy, size in field_objects:
        x_rel, y_rel = transform_to_roomba_frame(ox, oy)
        ax2.plot(x_rel, y_rel, 'o', color=get_color(size), alpha=0.7)
    ax2.legend()


# === Key Input Handler ===
# Forwards commands to the robot and mirrors turn commands into the local heading.
def on_key_press_with_heading(event):
    global roomba_heading
    if event.name == 'esc':
        sock.close()
        print('Program ended.')
        exit()
    elif event.name == 'm':
        # Clear the radar view, then forward 'm' to the robot
        degree.clear()
        rarr.clear()
        sizearr.clear()
        min_degrees.clear()
        max_degrees.clear()
        path_angles.clear()
        path_dists.clear()
        ax.clear()
        sock.send(b'm')
    elif event.name == 'n':
        # Clear the field map (local only)
        field_objects.clear()
        update_field_map()
    else:
        # Keep the dead-reckoned heading in sync with turn commands
        if event.name == 'a':
            roomba_heading = (roomba_heading + 1) % 360
        elif event.name == 'd':
            roomba_heading = (roomba_heading - 1) % 360
        elif event.name == '9':
            roomba_heading = (roomba_heading + 90) % 360
        elif event.name == '0':
            roomba_heading = (roomba_heading - 90) % 360
        sock.send(event.name.encode())


keyboard.on_press(on_key_press_with_heading)


# === Telemetry Receiver ===
# Background thread: updates the robot's position and the object lists.
def receive_data_with_map():
    global roomba_x, roomba_y
    while True:
        data = sock.recv(2048)
        if not data:
            break
        lines = data.decode(errors="ignore").strip().split('\n')
        for line in lines:
            print(f"📥 Received: {line}")

            # --- Odometry: advance the pose along the current heading ---
            if "Distance Moved" in line:
                try:
                    parts = line.split(',')
                    for part in parts:
                        if "Distance Moved" in part:
                            dist_cm = float(part.strip().split(':')[1])
                            dx = dist_cm * math.cos(math.radians(roomba_heading))
                            dy = dist_cm * math.sin(math.radians(roomba_heading))
                            roomba_x += dx
                            roomba_y += dy
                except Exception as e:
                    print("❌ Position update failed:", e)

            # --- Object record: add to radar and project onto the field map ---
            if line.startswith('Min Degree:'):
                try:
                    parts = line.split(',')
                    min_deg = int(parts[0].split()[2])
                    max_deg = int(parts[0].split()[4])
                    ir_cm = float(parts[1].split(':')[1].strip())
                    midpoint = int(parts[2].split(':')[1].strip())
                    width = float(parts[3].split(':')[1].strip())
                    size = 'small' if width < 5 else 'medium' if width < 15 else 'large'
                    angle_rad = math.radians(midpoint)
                    degree.append(angle_rad)
                    rarr.append(ir_cm)
                    sizearr.append(size)
                    min_degrees.append(math.radians(min_deg))
                    max_degrees.append(math.radians(max_deg))
                    path_angles.append(angle_rad)
                    path_dists.append(ir_cm)

                    # Project into the global frame
                    obj_dx, obj_dy = polar_to_cartesian_global(ir_cm, midpoint, roomba_heading)
                    obj_x = roomba_x + obj_dx
                    obj_y = roomba_y + obj_dy

                    # De-duplicate: skip if within 5 cm of an existing map point
                    is_duplicate = False
                    for existing_x, existing_y, _ in field_objects:
                        if math.hypot(existing_x - obj_x, existing_y - obj_y) < 5:
                            is_duplicate = True
                            break
                    if not is_duplicate:
                        field_objects.append((obj_x, obj_y, size))
                except Exception as e:
                    print("❌ Parse error:", line)
                    print("‼️ Exception:", e)


receive_thread = threading.Thread(target=receive_data_with_map)
receive_thread.start()


# === Radar Animation ===
def animate_radar(i):
    ax.clear()
    ax.set_rlabel_position(-22.5)
    ax.set_title("Roomba Radar System", va='bottom')
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_rmax(100)
    ax.scatter(0, 0, color='black', s=100, label='Roomba')
    ax.plot([0, math.pi], [17.5, 17.5], color='Red', linewidth=3.5, alpha=1.0, label='Roomba Width')
    max_dist = 0
    max_angle = 0
    for angle, dist, size, a_min, a_max in zip(degree, rarr, sizearr, min_degrees, max_degrees):
        # Beam line, size-coloured point, and label
        color = get_color(size)
        ax.plot([0, angle], [0, dist], linestyle='-', color='purple', linewidth=2.5, alpha=0.6)
        ax.scatter(angle, dist, color=color, s=80, alpha=0.8)
        angle_deg = round(math.degrees(angle))
        dist_txt = round(dist, 2)
        ax.text(angle, dist + 5, f"{angle_deg}°\n{dist_txt}cm\n{size}", fontsize=6, ha='center')

        # Arc showing the object's angular span at its distance
        width_arc = np.linspace(a_min, a_max, 30)
        width_radii = [dist] * len(width_arc)
        ax.plot(width_arc, width_radii, color='gray', linewidth=2.0, alpha=0.4)

        # Track the farthest object
        if dist > max_dist:
            max_dist = dist
            max_angle = angle
    if max_dist > 0:
        ax.plot([0, max_angle], [0, max_dist], linestyle='--', color='cyan', linewidth=2, label='Max Distance')
    if len(path_angles) > 1:
        ax.plot(path_angles, path_dists, linestyle=':', color='blue', linewidth=1, alpha=0.5)
    ax.legend(loc='upper right', fontsize=6)


# === Field Map Animation ===
def animate_field(i):
    update_field_map()


# === Run (both windows refresh every 500 ms) ===
ani_radar = animation.FuncAnimation(fig, animate_radar, interval=500)
ani_field = animation.FuncAnimation(fig2, animate_field, interval=500)
plt.show(block=True)
