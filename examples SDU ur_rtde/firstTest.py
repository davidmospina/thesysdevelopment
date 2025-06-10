import rtde_control
import time

# Initialize RTDE Control Interface
rtde_c = rtde_control.RTDEControlInterface("192.168.56.101")

# Initial joint positions
joint_positions = [0, 0, 0, 0, 0, 0]

# Increment value
increment = 0.1

# Loop to increment the first joint's position
try:
    while True:
        # Send the updated joint positions to the robot
        rtde_c.moveJ(joint_positions, 0.5, 0.3)
        
        # Increment the first joint's position
        joint_positions[0] += increment
        
        # Wait for 1 second
        time.sleep(0.003)
except KeyboardInterrupt:
    print("Program interrupted by user.")