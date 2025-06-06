#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import argparse  # Import argparse for command-line arguments
sys.path.append("..")
import logging
import threading
import time
import csv
import socket
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import matplotlib.pyplot as plt

# Initialize logging and configurations
logging.basicConfig(level=logging.INFO)
conf = rtde_config.ConfigFile("bridged_values.xml")
state_names, state_types = conf.get_recipe("robot_state")
tcp_names, tcp_types = conf.get_recipe("tcp_position_partner")
ft_names, ft_types = conf.get_recipe("force_torque_sensor_partner")
pos_sync_names, pos_sync_types = conf.get_recipe("pos_sync")

# Initialize connection variables
ROBOT_HOST_1 = "192.168.56.101"
ROBOT_HOST_2 = "192.168.56.102"
ROBOT_HOST_3 = "10.0.0.3"

ROBOT_PORT = 30004
PRIMARY_PORT = 30001
CSV_FILE = "tcp_data.csv"


forceIndex = 30
tcpIndex = 24
syncPosIndex = 24

# Initialize shared variables with thread synchronization
masterTCP = None
slaveTCP = None
keep_running = True
lock = threading.Lock()

# Function to safely update global state variables
def update_state(masterCon, slaveCon, inputsSlave,inputsMaster):
    global masterTCP, slaveTCP, keep_running

    stateMaster = masterCon.receive()
    stateSlave = slaveCon.receive()

    if stateMaster is None or stateSlave is None:
        keep_running = False
        return

    # Lock shared state
    with lock:
        masterTCP = stateMaster.actual_TCP_pose
        masterFT = stateMaster.actual_TCP_force
        masterSyncPosition = stateMaster.output_int_register_24
        slaveTCP= stateSlave.actual_TCP_pose
        slaveFT= stateSlave.actual_TCP_force
        slaveSyncPosition = stateSlave.output_int_register_24
    list_to_float_registers(inputsSlave, masterTCP, tcpIndex)
    list_to_float_registers(inputsSlave, masterFT, forceIndex)
    int_to_int_register(inputsSlave, masterSyncPosition, syncPosIndex)

    list_to_float_registers(inputsMaster, slaveTCP, tcpIndex)
    list_to_float_registers(inputsMaster, slaveFT, forceIndex)
    int_to_int_register(inputsMaster, slaveSyncPosition, syncPosIndex)

    # print(masterTCP)
    # print(slaveTCP)
    slaveCon.send(inputsSlave)
    masterCon.send(inputsMaster)
    keep_running = True

def list_to_float_registers(dic, list, index):
    for i in range(0, 6):
        regNum = index + i
        dic.__dict__[f"input_double_register_{regNum}"] = list[i]
    return dic


def int_to_int_register(dic, value, index):
   
    dic.__dict__[f"input_int_register_{index}"] = value
    return dic

def collect_and_save_data():
    global keep_running, masterTCP, slaveTCP
    start_time = time.time()

    # Open the CSV file for saving data (use 'a' mode to append)
    with open(CSV_FILE, mode='w', newline='') as file:  # 'a' mode to append data
        writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Master TCP Y", "Master TCP Z", "Slave TCP Y", "Slave TCP Zzz"])

        while keep_running:
            time.sleep(0.5)  # Wait for 0.5 seconds

            timestamp = time.time() - start_time  # Get time elapsed
            with lock:
                if masterTCP is not None and slaveTCP is not None:
                    print([timestamp, masterTCP[1], masterTCP[2], slaveTCP[1], slaveTCP[2]])
                    writer.writerow([timestamp, masterTCP[1], masterTCP[2], slaveTCP[1], slaveTCP[2]])
                    file.flush()

def plot_tcp_data(file_path):
    # Prepare lists to store data
    timestamps = []
    master_y = []
    master_z = []
    slave_y = []
    slave_z = []

    try:
        # Read the CSV file and extract the relevant data
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            # Check if the file is empty or only has the header
            first_row = next(reader, None)  # Read the first row (header)
            if first_row is None:
                print("CSV file is empty. No data to plot.")
                return
            # If there's data, continue reading and parsing it
            for row in reader:
                timestamps.append(float(row[0]))  # Timestamp (s)
                master_y.append(float(row[1]))   # Master TCP Y
                master_z.append(float(row[2]))   # Master TCP Z
                slave_y.append(float(row[3]))    # Slave TCP Y
                slave_z.append(float(row[4]))    # Slave TCP Z

        # Create a figure with two subplots (axes)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot Master vs Slave for Y values
        ax1.plot(timestamps, master_y, label='Master Y', color='blue')
        ax1.plot(timestamps, slave_y, label='Slave Y', color='red')
        ax1.set_title("Master vs Slave TCP Y")
        ax1.set_xlabel("Timestamp (s)")
        ax1.set_ylabel("TCP Y")
        ax1.legend()

        # Plot Master vs Slave for Z values
        ax2.plot(timestamps, master_z, label='Master Z', color='blue')
        ax2.plot(timestamps, slave_z, label='Slave Z', color='red')
        ax2.set_title("Master vs Slave TCP Z")
        ax2.set_xlabel("Timestamp (s)")
        ax2.set_ylabel("TCP Z")
        ax2.legend()

        # Adjust layout for better presentation
        plt.tight_layout()

        # Show the plots
        plt.show()

    except Exception as e:
        print(f"Error reading the CSV file: {e}")

# Main execution function
def main():

    global keep_running 
        # Argument parser to handle input parameters
    parser = argparse.ArgumentParser(description="Robot control script")
    parser.add_argument("robot_type", choices=["virtual", "real"], help="Choose 'virtual' or 'real' for the robot connection")
    parser.add_argument("--collection", action="store_true", help="Collect and save data to CSV")
    parser.add_argument("--plot", action="store_true", help="Plot data when interrupted")
    args = parser.parse_args()

    # Select the correct master connection based on the input parameter
    if args.robot_type == "real":
        master_host = ROBOT_HOST_3  # Use ROBOT_HOST_3 for "real"
    else:
        master_host = ROBOT_HOST_2  # Use ROBOT_HOST_2 for "virtual"
        
    try:
        slaveCon = rtde.RTDE(ROBOT_HOST_1, ROBOT_PORT)
        slaveCon.connect()
        print("Connected to Robot 1 successfully.")
        
        masterCon = rtde.RTDE(master_host, ROBOT_PORT)
        masterCon.connect()
        print(f"Connected to Robot 2 ({args.robot_type}) successfully.")

    except Exception as e:
        print(f"An error occurred: {e}")
        return

    # setup recipes
    slaveCon.send_output_setup(state_names, state_types)
    inputsSlave = slaveCon.send_input_setup(tcp_names + ft_names + pos_sync_names, tcp_types + ft_types + pos_sync_types)

    # setup recipes
    masterCon.send_output_setup(state_names, state_types)
    inputsMaster = masterCon.send_input_setup(tcp_names + ft_names + pos_sync_names, tcp_types + ft_types + pos_sync_types)

    # Start data collection in a separate thread
    if args.collection or args.plot:
        data_thread = threading.Thread(target=collect_and_save_data)
        data_thread.daemon = True
        data_thread.start()

    # Main loop to update robot state
    if not slaveCon.send_start() or not masterCon.send_start():
        
        sys.exit()

    try:
        while keep_running:
            update_state(masterCon, slaveCon, inputsSlave, inputsMaster)
            time.sleep(0.001)  # Add a short delay to avoid overloading the CPU
    except KeyboardInterrupt:
        print("\nInterrupted!")

        keep_running = False

        # Wait for the data collection thread to finish
        if args.plot or args.plot:
            data_thread.join()
        if args.plot:
            print("\nPlotting data...")
            plot_tcp_data(CSV_FILE)  # Call the plot function when interrupted
        sys.exit()

if __name__ == "__main__":
    main()
