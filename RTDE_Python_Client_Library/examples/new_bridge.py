import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import threading
import time

# RTDE Connection Parameters
ROBOT_IPS = ["10.0.0.2", "10.0.0.3", "10.0.0.4"]  # Add more robot IPs as needed
PORT = 30004
CONFIG_FILE = "rtde_config.xml"
MASTER_IP = "10.0.0.2"  # Define master robot IP

# Map partner IDs to robot IPs
PARTNER_IPS = {
    1: "10.0.0.3",  # Partner ID 1 -> Robot IP
    2: "10.0.0.4",  # Partner ID 2 -> Robot IP
    
}

# Define shared data structures for scalability
class RobotConnection:
    def __init__(self, ip):
        self.ip = ip
        self.master_ip = MASTER_IP
        self.is_master = self.ip == self.master_ip  
        self.running = True
        self.followerCon = {}

        self.config = rtde_config.ConfigFile(CONFIG_FILE)
        self.master_out = self.config.get_recipe("MASTER_OUT")
        self.master_in = self.config.get_recipe("MASTER_IN")
        self.follower_out = self.config.get_recipe("FOLLOWER_OUT")
        self.follower_in = self.config.get_recipe("FOLLOWER_IN")

    def connect(self):
        
        try:
            for partner_id, partner_ip in PARTNER_IPS.items():
                print(f"Partner ID: {partner_id}, Partner IP: {partner_ip}")
                self.followerCon[partner_id] = rtde.RTDE(self.partner_ip, PORT) #access them by doing self.followerCon[1]
                if not self.followerCon[partner_id].connect():
                    print(f"Failed to connect to Partner ID: {partner_id}, with IP: {partner_ip}")
                    return False
                print(f"Succesfully connected to Partner {partner_id}, with IP: {partner_ip}")
                self.followerCon[partner_id].get_controller_version()
                self.followerCon[partner_id].send_output_setup(self.follower_out)
                self.followerCon[partner_id].send_input_setup(self.follower_in)
            
            self.masterCon = rtde.RTDE(MASTER_IP, PORT)
            self.masterCon.connect()
            if not self.masterCon.connect():
                    print(f"Failed to connect to Partner ID: {partner_id}, with IP: {partner_ip}")
                    return False
            print(f"Connected to Master Robot, with IP {MASTER_IP}")
            self.masterCon.get_controller_version()
            self.masterCon.send_output_setup(self.master_out)
            self.masterCon.send_input_setup(self.master_in)

        except Exception as e:
            print(f"An error occurred: {e}")
            return
    
        return True
    

    def update_partner(self, state):
        # Get partner ID from the master (register 29)
        partner_id = state.output_int_register_29
        self.partner_ip = PARTNER_IPS.get(partner_id, None)  # Map ID to IP

        if not self.partner_ip:
            print(f"Invalid partner ID {partner_id} received.")

    def sync_data(self):
        while self.running:
            stateMaster = self.connection.receive()  # Always receive master's state
            if stateMaster is None:
                continue
            
            # If the thread is running on the master
            if self.is_master:
                # Identify the target follower and receive its state
                follower_connection = self.get_follower_connection()  # Method to get the right follower connection
                stateFollower = follower_connection.receive() if follower_connection else None

                if stateFollower:
                    self.link_registers(stateMaster, stateFollower)  # Pass both states to link registers
                
                self.connection.send(stateMaster)  # Send updated master state

            # If it's a follower thread
            elif self.partner_ip and self.ip == self.partner_ip:
                self.link_registers(stateMaster, self.connection.receive())  # Get master state, update follower
                self.connection.send(stateMaster)  # Send updated follower state

            time.sleep(0.002)  # 500 Hz sync rate


    def link_registers(self, state):
        inputsMaster = {}  # For master
        inputsFollower = {}  # For targeted follower
        
        if self.is_master:
            # Link registers from the follower's outputs to the master's inputs
            print(f"Linking registers from Follower {self.partner_ip} to Master {MASTER_IP}")
            inputsMaster = self.populate_master_input_dict(state)

        # If it's a follower thread and it has a partner IP, update the follower's registers
        elif self.partner_ip and self.ip == self.partner_ip:
            # Link registers from the master's outputs to the follower's inputs
            print(f"Linking registers from Master {MASTER_IP} to Follower {self.partner_ip}")
            inputsFollower = self.populate_follower_input_dict(state)

        else:
            print(f"No partner found for {self.ip}, skipping follower register update.")
        
        # send the updated state/dictionary to master and follower
        if self.is_master:
            self.connection.send(inputsMaster)  # Send to master
        elif self.partner_ip and self.ip == self.partner_ip:
            self.connection.send(inputsFollower) # Send the updated inputs only to the targeted follower

        print(f"Sent updated registers to Master {MASTER_IP} and Partner {self.partner_ip}.")

    def populate_master_input_dict(self, state):
        # Populate inputsMaster dictionary based on the follower's output registers
        inputsMaster = {}
                
        # index = followerState.register_number (this will assign the value from the follower register to the index)
        followerSyncPosition = state.output_int_register_24
        followerFunctionStatus = state.output_int_register_25
        followerIOreadconfi = state.output_bit_register_65
        followerIOreadstandard = state.output_bit_register_66
        followerIOreadanalog = state.output_double_register_37

        # Link integer registers
        for index, value in zip([24, 25], [followerSyncPosition, followerFunctionStatus]):
            setattr(inputsMaster, f"input_int_register_{index}", value)

        # Link double register 
        for index, value in zip([37], [followerIOreadanalog]):
            setattr(inputsMaster, f"input_double_register_{index}", value)

        # Link bit registers (booleans)
        for index, value in zip([65, 66], [followerIOreadconfi, followerIOreadstandard]):
            setattr(inputsMaster, f"input_bit_register_{index}", value)

        return inputsMaster
    

    def populate_follower_input_dict(self, state):
        # Populate inputsFollower dictionary based on the master's output registers
        inputsFollower = {}

        masterSyncPosition = state.output_int_register_24
        masterFunctionStatus = state.output_int_register_25
        masterIOlevel = state.output_bit_register_64
        masterIOoutputnum = state.output_int_register_27
        masterIOinputnum = state.output_int_register_28

        # Link integer registers
        for index, value in zip([24, 25, 27, 28], [masterSyncPosition, masterFunctionStatus, masterIOoutputnum, masterIOinputnum]):
            setattr(inputsFollower, f"input_int_register_{index}", value)

        # Link bit registers (booleans)
        for index, value in zip([64], [masterIOlevel]):
            setattr(inputsFollower, f"input_bit_register_{index}", value)

        # Link double registers
        list = [24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 37]
        self.float_reg_followerin(list, inputsFollower, state)

        return inputsFollower
    

    def float_reg_followerin(register_list, inputsFollower, stateMaster):
        for reg in register_list:
            setattr(inputsFollower, f"input_double_register_{reg}", getattr(stateMaster, f"output_double_register_{reg}", None))
        return inputsFollower

    
    def close(self):
        self.running = False
        self.connection.disconnect()

# Initialize and start threads for each robot
robots = [RobotConnection(ip) for ip in ROBOT_IPS]
threads = []

for robot in robots:
    if robot.connect():
        t = threading.Thread(target=robot.sync_data)
        t.start()
        threads.append(t)

try:
    for t in threads:
        t.join()
except KeyboardInterrupt:
    for robot in robots:
        robot.close()
    print("RTDE bridge stopped.")
