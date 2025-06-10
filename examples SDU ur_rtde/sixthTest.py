import rtde_receive
import rtde_control
import datetime
import math
import os
import psutil
import sys
import threading
import time

def getSegmentTarget(timestep, limit_min=-0.4499805051349124, limit_max=0.4499805051349124, freq=0.2):
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
    # Compute the midpoint and amplitude of the segment
    midpoint = (limit_max + limit_min) / 2
    amplitude = (limit_max - limit_min) / 2

    # Calculate the position using a sine wave
    # The sine wave is shifted and scaled to move between limit_min and limit_max
    position = midpoint + amplitude * math.sin(2 * math.pi * freq * timestep)
    return position


# Parameters
vel = 0.5
acc = 0.5
rtde_frequency = 500.0
dt = 1.0/rtde_frequency  # 2ms
flags = rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
ur_cap_port = 50002
robot_ip = "192.168.56.101"

lookahead_time = 0.1
gain = 600

# ur_rtde realtime priorities [from 0 to 99]
rt_receive_priority = 90
rt_control_priority = 85

rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.102", rtde_frequency, [], True, False, rt_receive_priority)
rtde_c = rtde_control.RTDEControlInterface("192.168.56.101", rtde_frequency, flags, ur_cap_port, rt_control_priority)

# Set application real-time priority
os_used = sys.platform
process = psutil.Process(os.getpid())
if os_used == "win32":  # Windows (either 32-bit or 64-bit)
    process.nice(psutil.REALTIME_PRIORITY_CLASS)
elif os_used == "linux":  # linux
    rt_app_priority = 80
    param = os.sched_param(rt_app_priority)
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
    except OSError:
        print("Failed to set real-time process scheduler to %u, priority %u" % (os.SCHED_FIFO, rt_app_priority))
    else:
        print("Process real-time priority set to: %u" % rt_app_priority)

time_counter = 0.0

global targetMiddle
# Get the current joint positions
targetMiddle = rtde_r.getActualTCPPose()

# Initialize targets
targetA = targetMiddle.copy()
targetA = -0.4499805051349124  # Modify the 2nd element
targetB = targetMiddle.copy()
targetB = 0.4499805051349124  # Modify the 2nd element

# Function to update the z-component of targets
def update_z_component():
    while True:
        try:
            # Update z-component dynamically
            targetMiddle = rtde_r.getActualTCPPose()
            print("inside of thread value:" + str(targetMiddle))
            time.sleep(0.002)  # Adjust update frequency as needed
        except Exception as e:
            print(f"Error in update thread: {e}")
            break

# Start the update thread
z_update_thread = threading.Thread(target=update_z_component, daemon=True)
z_update_thread.start()

# Main movement loop
actual_q = rtde_r.getActualQ()
rtde_c.moveJ(actual_q, 0.25, 0.5, False)

try:
    while True:
        t_start = rtde_c.initPeriod()
        x = getSegmentTarget(timestep=time_counter,)
        
        targetMiddle[1] = x 
        print("out of thread values:" + str(targetMiddle))

        rtde_c.servoL(targetMiddle, vel, acc, dt, lookahead_time, gain)
        rtde_c.waitPeriod(t_start)
        time_counter += dt

except KeyboardInterrupt:
    print("Program interrupted by user.")



