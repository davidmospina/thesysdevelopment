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
import math
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import csv
import socket
import pynput
from pynput.keyboard import Key, Listener
import time

logging.basicConfig(level=logging.INFO)

ROBOT_HOST_1 = "192.168.56.101"
ROBOT_HOST_2 = "192.168.56.102"
ROBOT_PORT = 30004
PRIMARY_PORT = 30001
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

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
setpSlave = slaveCon.send_input_setup(setp_names, setp_types)

# setup recipes
masterCon.send_output_setup(state_names, state_types)
setpMaster = masterCon.send_input_setup(setp_names, setp_types)



setpSlave.input_double_register_0 = 0
setpSlave.input_double_register_1 = 0
setpSlave.input_double_register_2 = 0
setpSlave.input_double_register_3 = 0
setpSlave.input_double_register_4 = 0
setpSlave.input_double_register_5 = 0

setpMaster.input_double_register_0 = 0
setpMaster.input_double_register_1 = 0
setpMaster.input_double_register_2 = 0
setpMaster.input_double_register_3 = 0
setpMaster.input_double_register_4 = 0
setpMaster.input_double_register_5 = 0

masterPointList = None
slavePointList = None
start_time = None

def setp_to_list(sp):
    sp_list = []
    for i in range(0, 6):
        sp_list.append(sp.__dict__["input_double_register_%i" % i])
    return sp_list


def list_to_setp(sp, list):
    for i in range(0, 6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

def getSegmentTarget(p, amp = 0.3, freq=0.2):

    p = p + freq*(2*3.1416/500)
    position = 0.69+amp*math.sin(p)
    return position,p

def getSegmentTargetConstantSpeed(relativeZ, amp, T, middlePoint, moving_up=True):
    # Define the rate of change per call
    dz = (2*amp) / T / 500  # 500 calls per second, duration T seconds
    
    # Update relativeZ: increase/decrease by dz
    if moving_up:
        relativeZ += dz  # Move upwards
    else:
        relativeZ -= dz  # Move downwards
    
    # If relativeZ exceeds the positive amplitude, reverse direction to decrease
    if relativeZ >= amp:
        relativeZ = amp  # Cap to the positive amplitude
        moving_up = False  # Change direction to moving downward
    
    # If relativeZ exceeds the negative amplitude, reverse direction to increase
    elif relativeZ <= -amp:
        relativeZ = -amp  # Cap to the negative amplitude
        moving_up = True  # Change direction to moving upward
    
    # Calculate the final z position
    z = middlePoint + relativeZ
    
    return z, relativeZ, moving_up

def updateRobots(keep_running, masterPointList, slavePointList):
    
    stateMaster = masterCon.receive()
    stateSlave = slaveCon.receive()
    if stateMaster is None or stateSlave is None:
        keep_running = False
        return keep_running    

    # Update the plot

    masterPointList = stateMaster.actual_TCP_pose
    slavePointList = stateSlave.actual_TCP_pose

    list_to_setp(setpSlave, masterPointList)
    # send new setpoint
    slaveCon.send(setpSlave)
    keep_running = True
    return keep_running, masterPointList, slavePointList  # Indicate that the loop should continue


# Initialize variables for Cartesian position
x, y, z = 0.0, 0.0, 0.0  # Initial position (in meters)
step = 0.01  # Step size for each key press

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

def on_press(key):
    global x, y, z

    try:
        if key == Key.up:  # Move +Y
            y += step
        elif key == Key.down:  # Move -Y
            y -= step
        elif key == Key.left:  # Move -X
            x -= step
        elif key == Key.right:  # Move +X
            x += step
        elif key.char == 's':  # Move -Z
            z -= step
        elif key.char == 'd':  # Move +Z
            z += step

        print(f"Target Cartesian Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    except AttributeError:
        pass

def on_release(key):
    if key == Key.esc:  # Exit program when Esc is pressed
        return False

# Start keyboard listener in a separate thread
listener = Listener(on_press=on_press, on_release=on_release)
listener.start()
#//////////////////////////////////////////////////////////////////////////// PLOT BLOCK
# Initialize lists to store y and z coordinates
state = None
y_data = []
z_data = []

def update():

    if state is None or state.actual_TCP_pose is None:
         return line,  # Skip update if state is not ready
    # Get new data from state.actual_TCP_pose

    y = state.actual_TCP_pose[1]
    z = state.actual_TCP_pose[2]

    y_data.append(y)
    z_data.append(z)

    # Update the data in the line plot
    line.set_data(y_data, z_data)
    ax.set_xlim(min(y_data) - 0.01, max(y_data) + 0.01)
    ax.set_ylim(min(z_data) - 0.01, max(z_data) + 0.01)

    return line,

# Create the figure and axis
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)

# Set up plot limits and labels
ax.set_xlim(-0.2, 0.2)  # Initial limits; they will auto-adjust
ax.set_ylim(0.4, 0.7)  # Adjust this according to expected range
ax.set_title('Real-Time Trajectory of Y and Z')
ax.set_xlabel('Y Coordinate')
ax.set_ylabel('Z Coordinate')
#///////////////////////////////////////////////////////////////////////////////////


# start data synchronization
if not slaveCon.send_start() or not masterCon.send_start():
    print("entre")
    sys.exit()
setpSlave.input_int_register_24 = 0
slaveCon.send(setpSlave)
setpMaster.input_int_register_24 = 0
masterCon.send(setpMaster)

stateMaster = masterCon.receive()
masterInitPosition = stateMaster.output_int_register_24

while not masterInitPosition:
    masterInitPosition = stateMaster.output_int_register_24

print("master in position")

setpSlave.input_int_register_24 = 1
updateRobots(keep_running, masterPointList, slavePointList)

stateSlave = slaveCon.receive()
slaveInitPosition = stateSlave.output_int_register_24

while not slaveInitPosition:
    stateSlave = slaveCon.receive()
    slaveInitPosition = stateSlave.output_int_register_24

setpMaster.input_int_register_24 = 1
masterCon.send(setpMaster)


print("slave in position")

csv_filename = "motion_data.csv"

with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Write the header (optional)
    writer.writerow(["Time (s)", "Y Position 101", "Y postion 102","Z Position 101", "Z postion 102"],)

    try:
        while keep_running:
            start_time = time.time()
            keep_running, masterPointList, slavePointList = updateRobots(keep_running, masterPointList, slavePointList)
            if not keep_running:
                break
            
            writer.writerow([start_time, masterPointList[1], slavePointList[1],masterPointList[2], slavePointList[2]])
            
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        # Ensure these actions are executed when the program stops
        # plt.show()
        setpSlave.input_int_register_24 = 0
        slaveCon.send(setpSlave)
        setpMaster.input_int_register_24 = 0
        masterCon.send(setpMaster)
        print("input registers to 0")
        slaveCon.send_pause()
        slaveCon.disconnect()
        masterCon.send_pause()
        masterCon.disconnect()
        # urscript_command()
        # send_urscript_command(urscript_command,ROBOT_HOST_2)
        # send_urscript_command(urscript_command,ROBOT_HOST_1)