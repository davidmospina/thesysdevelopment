import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

def pose_to_matrix(pose):
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
    return T
def plot_tcp_3d_view(file_path, follower_to_master_pose):
    """
    Plots TCP data:
    - Transforms follower TCPs into master frame
    - Shifts all poses so master TCP is at origin
    - Rotates both TCPs into the master TCP's local coordinate frame
    - Plots: X vs Y and Z vs Time
    - Also plots ideal X-Y trajectories of master and follower for reference
    - Computes DTW distance to quantify how close real trajectories are to ideal
    """
    import csv
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.spatial.transform import Rotation as R
    from fastdtw import fastdtw
    from scipy.spatial.distance import euclidean

    timestamps = []
    master_x, master_y, master_z = [], [], []
    follower_x, follower_y, follower_z = [], [], []

    try:
        # Transform follower base to master base
        T_follower_to_master = pose_to_matrix(follower_to_master_pose)

        # # Get inverse rotation of master TCP
        # tcp_rotvec_rad = np.deg2rad(master_tcp_pose[3:])  # convert degrees to radians
        # R_tcp_inv = R.from_rotvec(tcp_rotvec_rad).inv().as_matrix()

        # master_translation = np.array(master_tcp_pose[:3]) / 1000

        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            headers = next(reader, None)
            if headers is None:
                print("CSV file is empty. No data to plot.")
                return

            for row in reader:
                timestamps.append(float(row[0]))

                # --- Master TCP ---
                master_point = np.array([float(row[1]), float(row[2]), float(row[3])])
                
                master_x.append(master_point[0])
                master_y.append(master_point[1])
                master_z.append(master_point[2])

                # --- Follower TCP ---
                fx, fy, fz = float(row[4]), float(row[5]), float(row[6])
                follower_tcp = np.array([fx, fy, fz, 1.0])  # homogeneous

                transformed_tcp = T_follower_to_master @ follower_tcp
                
                follower_x.append(transformed_tcp[0])
                follower_y.append(transformed_tcp[1])
                follower_z.append(transformed_tcp[2])

        # --- Ideal Trajectories ---
        # aM = 0.07       # Master radius
        # aP = 0.0425     # Partner radius
        # wM = 2 * np.pi / 30  # Master angular speed (rad/s)
        # wP = 2 * np.pi / 20   # Partner angular speed (rad/s)
        # dt = 0.002
        # T_total = 30
        # t = np.arange(0, T_total, dt)

        # xM_ideal = aM * np.cos(wM * t)
        # yM_ideal = aM * np.sin(wM * t)
        # xP_ideal = xM_ideal + aP * np.cos(wP * t)
        # yP_ideal = yM_ideal + aP * np.sin(wP * t)

        # # --- DTW Distance Calculation ---
        # real_master_xy = list(zip(master_x, master_y))
        # ideal_master_xy = list(zip(xM_ideal, yM_ideal))
        # real_follower_xy = list(zip(follower_x, follower_y))
        # ideal_follower_xy = list(zip(xP_ideal, yP_ideal))

        # master_dtw_dist, _ = fastdtw(real_master_xy, ideal_master_xy, dist=euclidean)
        # # follower_dtw_dist, _ = fastdtw(real_follower_xy, ideal_follower_xy, dist=euclidean)

        # print(f"DTW Distance (Master Real vs Ideal):   {master_dtw_dist:.4f} meters")
        # print(f"DTW Distance (Follower Real vs Ideal): {follower_dtw_dist:.4f} meters")

        # # --- Plot 1: X vs Y (trajectory shape) ---
        # plt.figure(figsize=(8, 8))
        # plt.plot(master_x, master_y, label="Master X vs Y (Measured)", color="blue")
        # plt.plot(follower_x, follower_y, label="Follower X vs Y (Measured)", color="red")
        # # plt.plot(xM_ideal, yM_ideal, label="Ideal Master Trajectory", linestyle='--', linewidth=1.5, color = "yellow")
        # # plt.plot(xP_ideal, yP_ideal, label="Ideal Partner Trajectory", linestyle='--', linewidth=1.5)
        # plt.xlabel("X (m)")
        # plt.ylabel("Y (m)")
        # plt.title("TCP X vs Y (Measured vs Ideal Trajectories)")
        # plt.axis('equal')
        # plt.grid(True, linestyle='--', linewidth=0.5)
        # plt.legend()
        # plt.tight_layout()
        # plt.show()
        # --- Combined Plot: Z and Y vs Time ---
        fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Define the vertical line timestamp
        start_time = 9.52

        # Plot Z vs Time
        axs[0].plot(timestamps, master_z, label="Master Z over Time", color="blue")
        axs[0].plot(timestamps, follower_z, label="Partner Z over Time (Transformed)", color="red")
        axs[0].axvline(x=start_time, color='green', linestyle='--', linewidth=1)
        axs[0].text(start_time -1 , axs[0].get_ylim()[1]*0.95, "Parallel motion starts", color='black', fontsize=12)
        axs[0].set_ylabel("Z (m)")
        axs[0].set_title("TCP Z over Time (Master Base Frame)")
        axs[0].grid(True, linestyle='--', linewidth=0.5)
        axs[0].legend()

        # Plot Y vs Time
        axs[1].plot(timestamps, master_y, label="Master Y over Time", color="blue")
        axs[1].plot(timestamps, follower_y, label="Partner Y over Time (Transformed)", color="red")
        axs[1].axvline(x=start_time, color='green', linestyle='--', linewidth=1)
        axs[1].text(start_time - 1, axs[1].get_ylim()[1]*1.1, "Parallel motion starts", color='black', fontsize=12)
        axs[1].set_xlabel("Timestamp (s)")
        axs[1].set_ylabel("Y (m)")
        axs[1].set_title("TCP Y over Time (Master Base Frame)")
        axs[1].grid(True, linestyle='--', linewidth=0.5)
        axs[1].legend()

        plt.tight_layout()
        plt.show()



    except Exception as e:
        print(f"Error: {e}")





print("\nPlotting 3D-style TCP views...")
pose_follower_to_master = [0.6648,-0.4085,0.0015,0,0,3.1416]

master_tcp_pose = [385.86, -59.3, 377.83, 82.391331 , -144.9583, 144.55725 ]  # rotation in degrees


plot_tcp_3d_view("./tcp_data.csv", pose_follower_to_master)
