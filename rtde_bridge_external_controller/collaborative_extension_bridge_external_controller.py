import sys
import argparse  # Import argparse for command-line arguments
sys.path.append("..")
import logging
import threading
import time
import csv
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import matplotlib
# matplotlib.use('TkAgg')
import pygame
import time
import signal

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pprint
import math

CONFIG_FILE = "collaborative_extension.xml"

# Initialize logging and configurations
logging.basicConfig(level=logging.INFO)
config = rtde_config.ConfigFile(CONFIG_FILE)
master_out_names, master_out_types = config.get_recipe("MASTER_OUT")
master_in_names, master_in_types = config.get_recipe("MASTER_IN")
follower_out_names, follower_out_types, = config.get_recipe("FOLLOWER_OUT")
follower_in_names, follower_in_types  = config.get_recipe("FOLLOWER_IN")


# Initialize connection variables
ROBOT_HOST_1 = "192.168.56.101"
ROBOT_HOST_2 = "192.168.56.102"
ROBOT_HOST_3 = "10.0.0.3"
ROBOT_HOST_4 = "10.0.0.2"
ROBOT_PORT = 30004
PRIMARY_PORT = 30001
CSV_FILE = "tcp_data.csv"


forceIndex = 30
moveTargetIndex = 24
syncPosIndex = 24
functionIndex = 25

# Initialize shared variables with thread synchronization
masterTCP = None
followerTCP = None
keep_running = True
lock = threading.Lock()

def handle_sigint(signum, frame):
    global keep_running
    print("\nSIGINT received. Shutting down gracefully...")
    pygame.quit()
    keep_running = False

# # signal.signal(signal.SIGINT, handle_sigint)

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise Exception("No joystick/controller connected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Initialized Controller: {joystick.get_name()}")

# Scaling factors (adjust if needed)
MAX_LINEAR_SPEED = 3  # [m/s]
MAX_ROTATION_SPEED = 4.0 # [rad/s]

# Function to safely update global state variables
def update_state(masterCon, followerCon, inputsFollower,inputsMaster):
    global masterTCP, followerTCP, keep_running, followerQ, masterQ

    stateMaster = masterCon.receive()
    stateFollower = followerCon.receive()

    if stateMaster is None or stateFollower is None:
        keep_running = False
        return
    
    #SET HERE THE USED REGISTERS
    master_in_int = [24,25]
    master_in_bool = [65,66]
    master_in_float = [37]
    follower_in_int = [24, 25, 27, 28]
    follower_in_bool = [64]
    follower_in_float = [24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37] #38 to 43 also used

    # Lock shared state
    with lock:

        # UPDATE MASTER IN REGISTERS BASED ON FOLLOWER OUTPUTS
        followerTCP= stateFollower.actual_TCP_pose #needed for plot
        followerQ= stateFollower.actual_q #needed for plot

        link_int(master_in_int,stateFollower,inputsMaster)
        link_bool(master_in_bool,stateFollower,inputsMaster)
        link_float(master_in_float,stateFollower,inputsMaster)
        # link_vector(followerTCP, 38, 43, inputsMaster)

        # UPDATE FOLLOWER IN REGISTERS BASED ON MASTER OUTPUTS
        masterTCP = stateMaster.actual_TCP_pose #needed for plot
        masterQ = stateMaster.actual_q #needed for plot
       
        link_int(follower_in_int,stateMaster,inputsFollower)
        link_bool(follower_in_bool,stateMaster,inputsFollower)
        link_float(follower_in_float,stateMaster,inputsFollower)

        controller_speeds , a_button = read_controller()
        link_external_controller_inputs(inputsFollower,inputsMaster, controller_speeds, a_button)

    
    followerCon.send(inputsFollower)
    masterCon.send(inputsMaster)
    keep_running = True

def apply_deadzone(value, threshold=0.08):
    return value if abs(value) >= threshold else 0.0

def scale_input(x, scale=2.0):
    return math.copysign(abs(x) ** scale, x)


def read_controller():
    pygame.event.pump()


    # Example use inside read_controller():
    left_stick_x = scale_input(apply_deadzone(joystick.get_axis(0)))
    left_stick_y = scale_input(apply_deadzone(joystick.get_axis(1)))
    right_stick_x = scale_input(apply_deadzone(joystick.get_axis(3)))
    right_stick_y = scale_input(apply_deadzone(joystick.get_axis(4)))

    lt = (joystick.get_axis(2) + 1) / 2
    rt = (joystick.get_axis(5) + 1) / 2
    lt = scale_input(apply_deadzone(lt), scale=3.0)
    rt = scale_input(apply_deadzone(rt), scale=3.0)

    a_button = joystick.get_button(0)

    lb = joystick.get_button(4)
    rb = joystick.get_button(5)

    # Velocity mapping (no repeated deadzone!)
    vx = left_stick_x * MAX_LINEAR_SPEED
    vy = -left_stick_y * MAX_LINEAR_SPEED
    vz = (rt - lt) * MAX_LINEAR_SPEED

    rx = -right_stick_y * MAX_ROTATION_SPEED
    ry = right_stick_x * MAX_ROTATION_SPEED
    rz = (rb - lb) * MAX_ROTATION_SPEED

    speed = [vx, vy, vz, rx, ry, rz], a_button

    # print(speed)
    return speed

def link_int(reg_indexes,source,destination):
    for reg in reg_indexes:
        setattr(destination, f"input_int_register_{reg}", getattr(source, f"output_int_register_{reg}", None))
    return destination
    
def link_bool(reg_indexes,source,destination):
    for reg in reg_indexes:
        setattr(destination, f"input_bit_register_{reg}", getattr(source, f"output_bit_register_{reg}", None))
    return destination

def link_float(reg_indexes,source,destination):
    for reg in reg_indexes:
        setattr(destination, f"input_double_register_{reg}", getattr(source, f"output_double_register_{reg}", None))
    return destination

def link_vector(vector, low_index, high_index, destination):
    for i in range(low_index, high_index + 1):  # Iterate through the register range
        setattr(destination, f"input_double_register_{i}", vector[i - low_index])
    return destination

def link_external_controller_inputs(inputsFollower,inputsMaster,controller_speeds, a_button):
    inputsMaster.input_double_register_38 = controller_speeds[0]
    inputsMaster.input_double_register_39 = controller_speeds[1]
    inputsMaster.input_double_register_40 = controller_speeds[2]
    inputsMaster.input_double_register_41 = controller_speeds[3]
    inputsMaster.input_double_register_42 = controller_speeds[4]
    inputsMaster.input_double_register_43 = controller_speeds[5]
    inputsMaster.input_int_register_30 = a_button
    

    inputsFollower.input_double_register_38 = controller_speeds[0]
    inputsFollower.input_double_register_39 = controller_speeds[1]
    inputsFollower.input_double_register_40 = controller_speeds[2]
    inputsFollower.input_double_register_41 = controller_speeds[3]
    inputsFollower.input_double_register_42 = controller_speeds[4]
    inputsFollower.input_double_register_43 = controller_speeds[5]
    inputsFollower.input_int_register_30 = a_button



def collect_and_save_tcp_data():
    global keep_running, masterTCP, followerTCP
    start_time = time.time()

    # Open the CSV file for saving data (use 'w' mode to write)
    with open(CSV_FILE, mode='w', newline='') as file:  # 'w' mode to overwrite existing data
        writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Master TCP X", "Master TCP Y", "Master TCP Z", "Follower TCP X", "Follower TCP Y", "Follower TCP Z"])

        while keep_running:
            time.sleep(0.5)  # Wait for 0.5 seconds

            timestamp = time.time() - start_time  # Get time elapsed
            with lock:
                if masterTCP is not None and followerTCP is not None:
                    writer.writerow([timestamp, masterTCP[0], masterTCP[1], masterTCP[2], followerTCP[0], followerTCP[1], followerTCP[2]])
                    file.flush()

def plot_tcp_data(file_path):
    # Prepare lists to store data
    timestamps = []
    master_x = []
    master_y = []
    master_z = []
    follower_x = []
    follower_y = []
    follower_z = []

    try:
        # Read the CSV file and extract the relevant data
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            first_row = next(reader, None)  # Read the first row (header)
            if first_row is None:
                print("CSV file is empty. No data to plot.")
                return
            
            for row in reader:
                timestamps.append(float(row[0]))  # Timestamp (s)
                master_x.append(float(row[1]))   # Master TCP X
                master_y.append(float(row[2]))   # Master TCP Y
                master_z.append(float(row[3]))   # Master TCP Z
                follower_x.append(float(row[4])) # Follower TCP X
                follower_y.append(float(row[5])) # Follower TCP Y
                follower_z.append(float(row[6])) # Follower TCP Z

        # Create a figure with three subplots
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

        # Plot Master vs Follower for X values
        ax1.plot(timestamps, master_x, label='Master X', color='blue')
        ax1.plot(timestamps, follower_x, label='Follower X', color='red')
        ax1.set_title("Master vs Follower TCP X")
        ax1.set_xlabel("Timestamp (s)")
        ax1.set_ylabel("TCP X")
        ax1.legend()
        ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(1))  # Set grid line spacing to 1 second

        # Plot Master vs Follower for Y values
        ax2.plot(timestamps, master_y, label='Master Y', color='blue')
        ax2.plot(timestamps, follower_y, label='Follower Y', color='red')
        ax2.set_title("Master vs Follower TCP Y")
        ax2.set_xlabel("Timestamp (s)")
        ax2.set_ylabel("TCP Y")
        ax2.legend()
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax2.xaxis.set_major_locator(ticker.MultipleLocator(1))  # Set grid line spacing to 1 second

        # Plot Master vs Follower for Z values
        ax3.plot(timestamps, master_z, label='Master Z', color='blue')
        ax3.plot(timestamps, follower_z, label='Follower Z', color='red')
        ax3.set_title("Master vs Follower TCP Z")
        ax3.set_xlabel("Timestamp (s)")
        ax3.set_ylabel("TCP Z")
        ax3.legend()
        ax3.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax3.xaxis.set_major_locator(ticker.MultipleLocator(1))  # Set grid line spacing to 1 second

        # Adjust layout for better presentation
        plt.tight_layout()

        # Show the plots
        plt.show()

    except Exception as e:
        print(f"Error: {e}")

def collect_and_save_joint_data():
    global keep_running, masterQ, followerQ
    start_time = time.time()

    with open('joint_data.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write CSV header for 6 joints each
        writer.writerow(
            ["Timestamp (s)"] +
            [f"Master Joint {i+1}" for i in range(6)] +
            [f"Follower Joint {i+1}" for i in range(6)]
        )

        while keep_running:
            time.sleep(0.5)
            timestamp = time.time() - start_time

            with lock:
                if masterQ is not None and followerQ is not None:
                    row = [timestamp] + list(masterQ[:6]) + list(followerQ[:6])
                    writer.writerow(row)
                    file.flush()


def plot_joint_data(file_path):
    timestamps = []
    master_joints = [[] for _ in range(6)]
    follower_joints = [[] for _ in range(6)]

    try:
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            headers = next(reader, None)
            if headers is None:
                print("CSV file is empty. No data to plot.")
                return

            for row in reader:
                timestamps.append(float(row[0]))
                for i in range(6):
                    master_joints[i].append(float(row[1 + i])*180/3.1416)
                    follower_joints[i].append(float(row[7 + i])*180/3.1416)

        # Create one figure with 6 subplots stacked vertically
        fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)

        for i in range(6):
            ax = axes[i]
            ax.plot(timestamps, master_joints[i], label=f'Master Joint {i+1}', color='blue')
            ax.plot(timestamps, follower_joints[i], label=f'Follower Joint {i+1}', color='red')
            ax.set_ylabel(f'J{i+1} Pos')
            ax.set_title(f'Master vs Follower Joint {i+1}')
            ax.grid(True, which='both', linestyle='--', linewidth=0.5)
            ax.legend()
            ax.xaxis.set_major_locator(ticker.MultipleLocator(1))

        # Common x-label at the bottom
        axes[-1].set_xlabel("Timestamp (s)")
        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error: {e}")

def plot_tcp_3d_view(file_path):
    timestamps = []
    master_x = []
    master_y = []
    master_z = []
    follower_x = []
    follower_y = []
    follower_z = []

    try:
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            headers = next(reader, None)
            if headers is None:
                print("CSV file is empty. No data to plot.")
                return

            for row in reader:
                timestamps.append(float(row[0]))
                master_x.append(float(row[1]))   # Master X
                master_y.append(float(row[2]))   # Master Y
                master_z.append(float(row[3]))   # Master Z
                follower_x.append(float(row[4])) # Follower X
                follower_y.append(float(row[5])) # Follower Y
                follower_z.append(float(row[6])) # Follower Z

        # --- Plot 1: Y vs Z ---
        plt.figure(figsize=(8, 5))
        plt.plot(master_x, master_z, label="Master X vs Z", color="blue")
        plt.plot(follower_x, follower_z, label="Follower X vs Z", color="red")
        plt.xlabel("X")
        plt.ylabel("Z")
        plt.title("TCP X vs Z")
        plt.grid(True, linestyle='--', linewidth=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()

        # --- Plot 2: Time vs X ---
        plt.figure(figsize=(8, 5))
        plt.plot(timestamps, master_y, label="Master Y over Time", color="blue")
        plt.plot(timestamps, follower_y, label="Follower Y over Time", color="red")
        plt.xlabel("Timestamp (s)")
        plt.ylabel("Y")
        plt.title("TCP Y over Time")
        plt.grid(True, linestyle='--', linewidth=0.5)
        plt.legend()
        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error: {e}")

# Main execution function
def main():
    global keep_running

    # Argument parser to handle input parameters
    parser = argparse.ArgumentParser(description="Robot control script")
    parser.add_argument("robot_type", choices=["virtual", "real"], help="Choose 'virtual' or 'real' for the robot connection")
    parser.add_argument("--collection", action="store_true", help="Collect and save data to CSV")

    # Mutually exclusive group for selecting TCP or Joint plotting
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--tcp", action="store_true", help="Plot TCP data on exit")
    group.add_argument("--q", action="store_true", help="Plot joint position data on exit")
    group.add_argument("--v3d", action="store_true", help="Plot TCP Y vs Z and Z vs Time")

    args = parser.parse_args()

    # Select the correct master connection based on the input parameter
    if args.robot_type == "real":
        master_host = ROBOT_HOST_3
        follower_host = ROBOT_HOST_4
    else:
        master_host = ROBOT_HOST_2
        follower_host = ROBOT_HOST_1

    try:
        followerCon = rtde.RTDE(follower_host, ROBOT_PORT)
        followerCon.connect()
        print("Connected to Robot 1 successfully.")

        masterCon = rtde.RTDE(master_host, ROBOT_PORT)
        masterCon.connect()
        print(f"Connected to Robot 2 ({args.robot_type}) successfully.")
    except Exception as e:
        print(f"An error occurred: {e}")
        return

    # setup recipes for Follower
    followerCon.send_output_setup(follower_out_names, follower_out_types)
    inputsFollower = followerCon.send_input_setup(follower_in_names, follower_in_types)

    # setup recipes for Master
    masterCon.send_output_setup(master_out_names, master_out_types)
    inputsMaster = masterCon.send_input_setup(master_in_names, master_in_types)

    # Start data collection in a separate thread based on selected type
    if args.collection or args.tcp or args.q or args.v3d:
        if args.q:
            data_thread = threading.Thread(target=collect_and_save_joint_data)
        else:
            data_thread = threading.Thread(target=collect_and_save_tcp_data)

        data_thread.daemon = True
        data_thread.start()

    # Start robots
    if not followerCon.send_start() or not masterCon.send_start():
        sys.exit()

    try:
        while keep_running:
            update_state(masterCon, followerCon, inputsFollower, inputsMaster)
    finally:
        print("\nExiting...")

        inputsMaster.input_int_register_25 = 0
        masterCon.send(inputsMaster)
        inputsFollower.input_int_register_25 = 0
        followerCon.send(inputsFollower)

        # pygame.quit()

        time.sleep(0.01)

        if args.collection or args.tcp or args.q:
            data_thread.join(timeout=2.0)

        try:
            if args.tcp:
                print("\nPlotting TCP data...")
                plot_tcp_data(CSV_FILE)
            elif args.q:
                print("\nPlotting joint position data...")
                plot_joint_data("joint_data.csv")
            elif args.v3d:
                print("\nPlotting 3D-style TCP views...")
                plot_tcp_3d_view(CSV_FILE)
        except Exception as e:
            print(f"Plotting interrupted or failed: {e}")

        sys.exit()

if __name__ == "__main__":
    main()