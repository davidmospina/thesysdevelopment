import rtde_receive
import rtde_control
import datetime
import math
import os
import psutil
import sys
import threading
import time

def getSegmentTarget(timestep, limit_min=-0.299805051349124, limit_max=0.299805051349124, freq=1/10):
    """
    Calculates the position along a 1D segment, starting from the middle, oscillating back and forth between two limits.

    Parameters:
    - timestep: Current time step (float).
    - limit_min: The minimum value of the segment (float).
    - limit_max: The maximum value of the segment (float).
    - freq: Frequency of one complete back-and-forth motion in Hz (float). Default is 0.2 Hz (5 seconds for one cycle).

    Returns:
    - Position along the segment (float).
    """
    midpoint = (limit_max + limit_min) / 2
    amplitude = (limit_max - limit_min) / 2
    position = midpoint + amplitude * math.sin(2 * math.pi * freq * timestep)
    return position

# Parameters
vel = 0.15
acc = 0.2
rtde_frequency = 500.0
dt = 1.0 / rtde_frequency  # 2ms
flags = rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
robot_ip = "192.168.56.101"

lookahead_time = 0.1
gain = 600

# ur_rtde realtime priorities
rt_receive_priority = 90
rt_control_priority = 85

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.102", rtde_frequency, [], True, False, rt_receive_priority) # IP; freq; variable; verbose/logging; upper registers; priority
rtde_c = rtde_control.RTDEControlInterface("192.168.56.101", rtde_frequency, flags, ur_cap_port, rt_control_priority) # IP; freq; flags; URCap Port; priority; verbose; upper registers

# Set application real-time priority
os_used = sys.platform
process = psutil.Process(os.getpid())
if os_used == "win32":
    process.nice(psutil.REALTIME_PRIORITY_CLASS)
elif os_used == "linux":
    rt_app_priority = 80
    param = os.sched_param(rt_app_priority)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
    except OSError:
        print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
    else:
        print("Process real-time priority set to: %u" % rt_app_priority)

time_counter = 0.0

# # Get the current joint positions
# lock = threading.Lock()

# Initialize targets
targetMiddle = rtde_r.getActualTCPPose()
targetA = targetMiddle.copy()
targetA[1] = -0.299805051349124
targetB = targetMiddle.copy()
targetB[1] = 0.299805051349124

# Function to update the z-component of targets
# def update_z_component():
#     while True:
#         try:
#             new_pose = rtde_r.getActualTCPPose()
#             with lock:  # Acquire the lock
#                 # Update the elements of targetMiddle in-place
#                 for i in range(len(targetMiddle)):
#                     targetMiddle[i] = new_pose[i]
#                 print("inside of thread value:" + str(targetMiddle))
#             time.sleep(0.002)  # Adjust update frequency as needed
#         except Exception as e:
#             print(f"Error in update thread: {e}")
#             break

# Start the update thread
# z_update_thread = threading.Thread(target=update_z_component, daemon=True)
# z_update_thread.start()

# Main movement loop
actual_q = rtde_r.getActualQ() #Read actual joint positions from robot 102
rtde_c.moveJ(actual_q, 0.25, 0.5, False) #Initial Movement, code waits until it reaches

print("Both robots at the starting positions")
time.sleep(5)

try:
    while True:
        t_start = rtde_c.initPeriod() #start of current control cycle
        targetMiddle = rtde_r.getActualTCPPose()
        
        x = getSegmentTarget(timestep=time_counter)
        

        targetMiddle[1] = x  #change Y coordinate
        #print("out of thread values:" + str(targetMiddle))

        rtde_c.servoL(targetMiddle, vel, acc, dt, lookahead_time, gain)  #dt = expected duration of control cycle
        rtde_c.waitPeriod(t_start)  #wait until current cycle reaches desired dt (based on t_start)
        time_counter += dt

except KeyboardInterrupt:
    print("Program interrupted by user.")




#Launch along test 4. Robot 101, the ones the moves horizontally, moves erraticatly: Maybe vertical movement of robot 101 is to fast to acomplish, maybe the refresh ratio of postion from 101 readed by 102 is to slows,  maybe the interuption of movement is not the correct -> need to check stopj (or simimilar) from the API of SDU