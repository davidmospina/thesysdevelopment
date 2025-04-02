import sys
sys.path.append("..")

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time

# Define the RTDE configuration file (assuming you have one)
# It defines which registers (e.g., output registers) will be used
ROBOT_IP = "10.0.0.3"  # Replace with your robot's IP address
PORT = 30004  # Default RTDE port

# Connect to the robot using RTDE
con = rtde.RTDE(ROBOT_IP, PORT)

# Define in one go all registers:
config_file = "/home/pobf/Desktop/config.xml"  # Replace with your RTDE config file path
conf = rtde_config.ConfigFile(config_file)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("PHC")

#output_names, output_types = conf.get_output_names(), conf.get_output_types()
#con.send_output_setup(output_names, output_types)  # Set up the output registers


# Start the connection to the robot
con.connect()
con.send_output_setup(setp_names, setp_types)
setp = con.send_input_setup(setp_names, setp_types)


# start data synchronization
if not con.send_start():
    sys.exit()

# Reset input_double_register_0 to 0.0
#setp.input_double_register_0 = 0.0
#con.send(setp)  # Send the reset value to the robot
#print("Reset input_double_register_0 to 0.0")

# Retrieve current register values (output register values)
first = con.receive()
# Print the initial register value
print(f"Initial value of input_double_register_0: {first.input_double_register_0}")

# Set a new value
setp.input_double_register_0 = 9.9
con.send(setp)  # Send the updated value to the robot
#print(f"Updated input_double_register_0 to: {setp.input_double_register_0}")

# Wait for a while to see the effect
time.sleep(1)

# Retrieve the updated register value
update = con.receive()
print(f"Current value of input_double_register_0: {update.input_double_register_0}")

# Optionally disconnect after use
con.disconnect()