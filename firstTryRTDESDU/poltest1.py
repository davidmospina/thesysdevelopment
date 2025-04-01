import sys
sys.path.append("..")

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time

# Define the RTDE configuration file (assuming you have one)
# It defines which registers (e.g., output registers) will be used
ROBOT_IP = "192.168.56.101"  # Replace with your robot's IP address
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
con.send_output_setup(state_names, state_types)

# start data synchronization
if not con.send_start():
    sys.exit()

test = con.receive()
print(test)

test.output_double_register_0 = 1737475507.207586086

#output_names = ["output_double_register_24"]  # Make sure the register name is correct
#output_types = ["DOUBLE"]
#con.send_output_setup(output_names, output_types)

#input_names = ["input_double_register_24"]  # Register for input (reading)
#input_types = ["DOUBLE"]  # Type for the input register
#con.send_input_setup(input_names, input_types)




con.send(test)

output_data = con.receive()

#received_clock_value = output_data["input_double_register_24"] #or alternatively check output_double_register_24

print(f"Clock Value Stored: 1737475507.207586086 ns")
#print(f"Clock Value Read Back: {received_clock_value} ns")

# Close the connection
con.disconnect()
