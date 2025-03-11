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
# logging.basicConfig(level=logging.INFO)

ROBOT_HOST = "192.168.56.101"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

# get controller version
con.get_controller_version()

# setup recipes
con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)

# Setpoints to move the robot to
setpList = [0.286,0,0.69,0,1.57,0]

middlePoint = setpList[2]


setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = middlePoint
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0


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

state = None

# Initialize lists to store y and z coordinates
y_data = []
z_data = []

def update():

    if state is None or state.actual_TCP_pose is None:
         return line,  # Skip update if state is not ready
    # print(str(state.actual_TCP_pose))
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


# start data synchronization
if not con.send_start():
    sys.exit()
p = 0
amp = 0.3  # Amplitude
T = 30  # Duration of the motion (seconds)
  # Middle point (usually 0)

relativeZ = 0  # Initial position (starting at middle point)
moving_up = True  # Start moving upwards


# Target frequency in Hz
frequency = 500
cycle_time = 1.0 / frequency

con.send(setp)
csv_filename = "motion_data.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Write the header (optional)
    writer.writerow(["Time (s)", "Z Position"])

    try:
        while keep_running:
            start_time = time.time()

            # Update state from robot
            state = con.receive()

            if state is None:
                break

            # Update the plot
            update()

            # # moving following a sin wave
            # setpList[2], p = getSegmentTarget(p, amp=0.3, freq=0.1)
            
            # moving folowing a linear function
            setpList[2] , relativeZ, moving_up = getSegmentTargetConstantSpeed(relativeZ, amp, T, middlePoint, moving_up)

            list_to_setp(setp, setpList)

            # send new setpoint
            con.send(setp)


            writer.writerow([start_time, setpList[2]])

            elapsed_time = time.time() - start_time
            sleep_time = cycle_time - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                logging.warning("Loop overrun: execution time exceeded cycle time.")
    except KeyboardInterrupt:
        logging.info("Program interrupted by user.")
    finally:
        # Ensure these actions are executed when the program stops
        setp.input_double_register_0 = 0
        setp.input_double_register_1 = 0
        setp.input_double_register_2 = 0.69
        setp.input_double_register_3 = 0
        setp.input_double_register_4 = 0
        setp.input_double_register_5 = 0
        con.send(setp)
        plt.show()
        con.send_pause()
        con.disconnect()