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

sys.path.append("..")
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import csv
import socket
import concurrent.futures
import time
import threading

logging.basicConfig(level=logging.INFO)

ROBOT_HOST_1 = "192.168.56.101"
ROBOT_HOST_2 = "192.168.56.102"
ROBOT_PORT = 30004
PRIMARY_PORT = 30001
config_filename = "bridged_values.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("robot_state")
tcp_names, tcp_types = conf.get_recipe("tcp_position_partner")
ft_names, ft_types = conf.get_recipe("force_torque_sensor_partner")
pos_sync_names, pos_sync_types = conf.get_recipe("pos_sync")

forceIndex = 30
tcpIndex = 24
syncPosIndex = 24

try:
    slaveCon = rtde.RTDE(ROBOT_HOST_1, ROBOT_PORT)
    slaveCon.connect()
    print("Connected to Robot 1 successfully.")
    
    masterCon = rtde.RTDE(ROBOT_HOST_2, ROBOT_PORT)
    masterCon.connect()
    print("Connected to Robot 2 successfully.")
except Exception as e:
    print(f"An error occurred: {e}")


# get controller version
slaveCon.get_controller_version()

# get controller version
masterCon.get_controller_version()

# setup recipes
slaveCon.send_output_setup(state_names, state_types)
inputsSlave = slaveCon.send_input_setup(tcp_names + ft_names + pos_sync_names, tcp_types + ft_types + pos_sync_types)

# setup recipes
masterCon.send_output_setup(state_names, state_types)
inputsMaster = masterCon.send_input_setup(tcp_names + ft_names + pos_sync_names, tcp_types + ft_types + pos_sync_types)

start_time = None
masterTCP = None
slaveTCP = None
CSV_FILE = "tcp_data.csv"

def list_to_float_registers(dic, list, index):
    for i in range(0, 6):
        regNum = index + i
        dic.__dict__[f"input_double_register_{regNum}"] = list[i]
    return dic


def int_to_int_register(dic, value, index):
   
    dic.__dict__[f"input_int_register_{index}"] = value
    return dic


def updateRobotsState():
    
    stateMaster = masterCon.receive()
    stateSlave = slaveCon.receive()
    if stateMaster is None or stateSlave is None:
        keep_running = False
        return keep_running    

    masterTCP = stateMaster.actual_TCP_pose
    masterFT = stateMaster.actual_TCP_force
    masterSyncPosition = stateMaster.output_int_register_24
    slaveTCP= stateSlave.actual_TCP_pose
    slaveFT= stateSlave.actual_TCP_force
    slaveSyncPosition = stateSlave.output_int_register_24

    # print("register master: " + str(stateMaster.output_int_register_24) + " register slave: " + str(stateSlave.output_int_register_24))

    list_to_float_registers(inputsSlave, masterTCP, tcpIndex)
    list_to_float_registers(inputsSlave, masterFT, forceIndex)
    int_to_int_register(inputsSlave, masterSyncPosition, syncPosIndex)

    list_to_float_registers(inputsMaster, slaveTCP, tcpIndex)
    list_to_float_registers(inputsMaster, slaveFT, forceIndex)
    int_to_int_register(inputsMaster, slaveSyncPosition, syncPosIndex)


    slaveCon.send(inputsSlave)
    masterCon.send(inputsMaster)
    keep_running = True


def cleanRobotsInputRegister():

    stateMaster = masterCon.receive()
    stateSlave = slaveCon.receive()
    if stateMaster is None or stateSlave is None:
        keep_running = False
        return keep_running    
    
    
    
    emptyList = [0,0,0,0,0,0]
    list_to_float_registers(inputsSlave, emptyList, tcpIndex)
    list_to_float_registers(inputsSlave, emptyList, forceIndex)
    int_to_int_register(inputsSlave, 0, syncPosIndex)

    list_to_float_registers(inputsMaster, emptyList, tcpIndex)
    list_to_float_registers(inputsMaster, emptyList, forceIndex)
    int_to_int_register(inputsMaster, 0, syncPosIndex)

    slaveCon.send(inputsSlave)
    masterCon.send(inputsMaster)
    stateMaster = masterCon.receive()
    stateSlave = slaveCon.receive()
    # print("register master: " + str(stateMaster.output_int_register_24) + " register slave: " + str(stateSlave.output_int_register_24))

       

    keep_running = True
    return keep_running 

def update_state_in_thread():
    keep_running = True
    while keep_running:
        keep_running = updateRobotsState()  # Call the function to update states
        # logging.info("State updated")

# Start the background thread to update states
def start_thread():

    threading.Thread(target=collect_and_save_data, daemon=True).start()

    with concurrent.futures.ThreadPoolExecutor() as executor:
        # Use an infinite loop to resubmit the task
        while True:
            future = executor.submit(update_state_in_thread)
            future.result()  # Wait for the result of the task

def send_urscript_command(command: str, robotIP):
    """
    This function takes the URScript command defined above, 
    connects to the robot server, and sends 
    the command to the specified port to be executed by the robot.

    Args:
        command (str): URScript command
        
    Returns: 
        None
    """
    try:
        # Create a socket connection with the robot IP and port number defined above
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((robotIP, PRIMARY_PORT))

        # Appends new line to the URScript command (the command will not execute without this)
        command = command+"\n"
        
        # Send the command
        s.sendall(command.encode('utf-8'))
        
        # Close the connection
        s.close()

    except Exception as e:
        print(f"An error occurred: {e}")

def init_csv():
    with open(CSV_FILE, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp (s)", "Master TCP Y", "Master TCP Z", "Slave TCP Y", "Slave TCP Z"])

def save_to_csv(timestamp, masterTCP, slaveTCP):
    print("hello save_to_csv")
    with open(CSV_FILE, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, masterTCP[1], masterTCP[2], slaveTCP[1], slaveTCP[2]])

def collect_and_save_data():
    print("hello collect_and_save_data")

    start_time = time.time()  # Store program start time
    while keep_running:
        time.sleep(0.5)  # Wait for 0.5 seconds
        
        timestamp = time.time() - start_time  # Get time elapsed since the start of the program
        print("hello collect_and_save_data while keep_running")
        # Check if the robots have valid data
        print("masterTCP: " + str(masterTCP) + " slaveTCP: " + str(slaveTCP))
        if masterTCP is not None and slaveTCP is not None:
            print("hello collect_and_save_data while keep_running  if masterTCP is not None and slaveTCP is not None:")
            save_to_csv(timestamp, masterTCP, slaveTCP)

if __name__ == "__main__":

    init_csv()  # Initialize the CSV file with headers

        # start data synchronization
    if not slaveCon.send_start() or not masterCon.send_start():
        
        sys.exit()
    # cleanRobotsInputRegister()
    future = start_thread()  # Start the background thread for state updates
    
    auxState = True
    while True:

        auxState = not auxState
        

