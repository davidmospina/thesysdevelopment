import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R

def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
    return T

def plot_tcp_3d_view(file_path, follower_to_master_pose, master_tcp_pose):
    # Load CSV
    df = pd.read_csv(file_path)

    # Transform follower base to master base
    T_follower_to_master = pose_to_matrix(follower_to_master_pose)

    # Get inverse rotation of master TCP
    tcp_rotvec_rad = np.deg2rad(master_tcp_pose[3:])  # convert degrees to radians
    R_tcp_inv = R.from_rotvec(tcp_rotvec_rad).inv().as_matrix()
    master_translation = np.array(master_tcp_pose[:3])

    master_x, master_y, master_z = [], [], []
    follower_x, follower_y, follower_z = [], [], []

    for _, row in df.iterrows():
        # --- Master TCP ---
        master_point = np.array([row["Master TCP X"], row["Master TCP Y"], row["Master TCP Z"]])
        shifted_master = master_point - master_translation
        rotated_master = R_tcp_inv @ shifted_master
        master_x.append(rotated_master[0])
        master_y.append(rotated_master[1])
        master_z.append(rotated_master[2])

        # --- Follower TCP ---
        follower_tcp = np.array([row["Follower TCP X"], row["Follower TCP Y"], row["Follower TCP Z"], 1.0])
        transformed_tcp = T_follower_to_master @ follower_tcp
        shifted_follower = transformed_tcp[:3] - master_translation
        rotated_follower = R_tcp_inv @ shifted_follower
        follower_x.append(rotated_follower[0])
        follower_y.append(rotated_follower[1])
        follower_z.append(rotated_follower[2])

    # --- Ideal Trajectories ---
    aM = 0.07       # Master radius
    aP = 0.0425     # Partner radius
    wM = 2 * np.pi / 30  # Master angular speed (rad/s)
    wP = 2 * np.pi / 5   # Partner angular speed (rad/s)
    dt = 0.002
    T_total = 30
    t = np.arange(0, T_total, dt)

    xM_ideal = aM * np.cos(wM * t)
    yM_ideal = aM * np.sin(wM * t)
    xP_ideal = xM_ideal + aP * np.cos(wP * t)
    yP_ideal = yM_ideal + aP * np.sin(wP * t)

    # --- Plot 1: X vs Y (trajectory shape) ---
    plt.figure(figsize=(8, 8))
    plt.plot(master_x, master_y, label="Master X vs Y (Measured)", color="blue")
    plt.plot(follower_x, follower_y, label="Follower X vs Y (Transformed)", color="red")
    plt.plot(xM_ideal, yM_ideal, label="Ideal Master Trajectory", linestyle='--', linewidth=1.5)
    plt.plot(xP_ideal, yP_ideal, label="Ideal Partner Trajectory", linestyle='--', linewidth=1.5)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("TCP X vs Y (Measured vs Ideal Trajectories)")
    plt.axis('equal')
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # --- Plot 2: Z vs Time ---
    plt.figure(figsize=(8, 5))
    plt.plot(df["Timestamp (s)"], master_z, label="Master Z over Time", color="blue")
    plt.plot(df["Timestamp (s)"], follower_z, label="Follower Z over Time (Transformed)", color="red")
    plt.xlabel("Timestamp (s)")
    plt.ylabel("Z (m)")
    plt.title("TCP Z over Time (Measured Only, Local TCP Frame)")
    plt.grid(True, linestyle='--', linewidth=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()

# Example usage:
pose_follower_to_master = [0.6816794, -0.3941288, 0.002, 0.0, 0.0, 3.1410]
master_tcp_pose = [0.38586, -0.593, 0.37783, 82.391331, -145.13021, 144.55725]  # rotation in degrees
plot_tcp_3d_view("tcp_data.csv", pose_follower_to_master, master_tcp_pose)
