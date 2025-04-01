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
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import pprint

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
masterTCPArray = None
followerTCP = None
keep_running = True
lock = threading.Lock()

# Function to safely update global state variables
def update_state(masterCon, followerCon, inputsFollower,inputsMaster):
    global masterTCPArray, followerTCP, keep_running

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
    follower_in_float = [24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 37] #38 to 43 also used

    # Lock shared state
    with lock:

        # UPDATE MASTER IN REGISTERS BASED ON FOLLOWER OUTPUTS
        followerTCP= stateFollower.actual_TCP_pose #needed for plot

        link_int(master_in_int,stateFollower,inputsMaster)
        link_bool(master_in_bool,stateFollower,inputsMaster)
        link_float(master_in_float,stateFollower,inputsMaster)
        link_vector(followerTCP, 38, 43, inputsMaster)

        # UPDATE FOLLOWER IN REGISTERS BASED ON MASTER OUTPUTS
        masterTCPArray = stateMaster.actual_TCP_pose #needed for plot
       
        link_int(follower_in_int,stateMaster,inputsFollower)
        link_bool(follower_in_bool,stateMaster,inputsFollower)
        link_float(follower_in_float,stateMaster,inputsFollower)

    
    followerCon.send(inputsFollower)
    masterCon.send(inputsMaster)
    keep_running = True


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



def collect_and_save_data():
    global keep_running, masterTCPArray, followerTCP
    start_time = time.time()

    # Open the CSV file for saving data (use 'a' mode to append)
    with open(CSV_FILE, mode='w', newline='') as file:  # 'a' mode to append data
        writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Master TCP Y", "Master TCP Z", "Follower TCP Y", "Follower TCP Zzz"])

        while keep_running:
            time.sleep(0.5)  # Wait for 0.5 seconds

            timestamp = time.time() - start_time  # Get time elapsed
            with lock:
                if masterTCPArray is not None and followerTCP is not None:
                    # print([timestamp, masterTCPArray[1], masterTCPArray[2], followerTCP[1], followerTCP[2]])
                    writer.writerow([timestamp, masterTCPArray[1], masterTCPArray[2], followerTCP[1], followerTCP[2]])
                    file.flush()

def plot_tcp_data(file_path):
    # Prepare lists to store data
    timestamps = []
    master_y = []
    master_z = []
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
                master_y.append(float(row[1]))   # Master TCP Y
                master_z.append(float(row[2]))   # Master TCP Z
                follower_y.append(float(row[3])) # Follower TCP Y
                follower_z.append(float(row[4])) # Follower TCP Z

        # Create a figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # Plot Master vs Follower for Y values
        ax1.plot(timestamps, master_y, label='Master Y', color='blue')
        ax1.plot(timestamps, follower_y, label='Follower Y', color='red')
        ax1.set_title("Master vs Follower TCP Y")
        ax1.set_xlabel("Timestamp (s)")
        ax1.set_ylabel("TCP Y")
        ax1.legend()
        ax1.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Set grid line spacing to 1 second
        ax1.xaxis.set_major_locator(ticker.MultipleLocator(1))  

        # Plot Master vs Follower for Z values
        ax2.plot(timestamps, master_z, label='Master Z', color='blue')
        ax2.plot(timestamps, follower_z, label='Follower Z', color='red')
        ax2.set_title("Master vs Follower TCP Z")
        ax2.set_xlabel("Timestamp (s)")
        ax2.set_ylabel("TCP Z")
        ax2.legend()
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Set grid line spacing to 1 second
        ax2.xaxis.set_major_locator(ticker.MultipleLocator(1))  

        # Adjust layout for better presentation
        plt.tight_layout()

        # Show the plots
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
    parser.add_argument("--plot", action="store_true", help="Plot data when interrupted")
    args = parser.parse_args()

    # Select the correct master connection based on the input parameter
    if args.robot_type == "real":
        master_host = ROBOT_HOST_3  # Use ROBOT_HOST_3 for "real"
        follower_host = ROBOT_HOST_4
    else:
        master_host = ROBOT_HOST_2  # Use ROBOT_HOST_2 for "virtual"
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

    # Start data collection in a separate thread
    if args.collection or args.plot:
        data_thread = threading.Thread(target=collect_and_save_data)
        data_thread.daemon = True
        data_thread.start()

    # Main loop to update robot state
    if not followerCon.send_start() or not masterCon.send_start():
        
        sys.exit()

    

    try:
        while keep_running:
            update_state(masterCon, followerCon, inputsFollower, inputsMaster)
            time.sleep(0.001)  # Add a short delay to avoid overloading the CPU
    except KeyboardInterrupt:
        print("\nInterrupted!")

        keep_running = False

        #CLEANUP PHASE + DEBUGGING (add here the registers that should be reset upon bridge shutdown) 
        inputsMaster.input_int_register_24 = 0
        masterCon.send(inputsMaster)  
        inputsFollower.input_int_register_24 = 0
        followerCon.send(inputsFollower)  

        time.sleep(0.01) #allow time to actually change registers


        # Wait for the data collection thread to finish
        if args.plot or args.plot:
            data_thread.join()
        if args.plot:
            print("\nPlotting data...")
            plot_tcp_data(CSV_FILE)  # Call the plot function when interrupted
        sys.exit()

if __name__ == "__main__":
    main()