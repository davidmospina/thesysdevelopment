import socket
import csv
import threading

# Server setup
host = '0.0.0.0'  # Listen on all available interfaces
port = 32769
buffer_size = 1024

# CSV file setup
csv_file_master = "mastercase1.csv"
csv_file_follower = "followercase1.csv"

# Check if the CSV files exist and create them with headers if not
def create_csv(file_name):
    try:
        with open(file_name, mode='r', newline='') as file:
            pass 
    except FileNotFoundError:
        with open(file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["sec", "nsec", "X", "Y", "Z", "Rx", "Ry", "Rz", "VX", "VY", "VZ"])

# Create new client handler to receive data
def handle_client(conn, addr):
    print(f"[SERVER] Connected by {addr}")

    # Determine client type based on known IP (master or partner)
    if addr[0] == '10.0.0.3':
        csv_file = csv_file_master
    elif addr[0] == '10.0.0.2':
        csv_file = csv_file_follower
    else:
        print(f"[SERVER] Unrecognized IP {addr[0]}. Closing connection.")
        conn.close()
        return

    # Open the corresponding CSV file in append mode
    with open(csv_file, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        try:
            while True:
                data = conn.recv(buffer_size)  # Receive data from client
                if not data:
                    print(f"[SERVER] Connection closed by {addr[0]}.")
                    break
                
                # Decode the received data 
                received_data = data.decode('utf-8').strip() 
                
                # Split the received data
                lines = received_data.strip().split(r'\q') 
                for line in lines:
                    if line:  # Avoid empty lines
                        row = line.split(',')  # Split each line into values
                        writer.writerow(row)  # Write the row to CSV
        except KeyboardInterrupt:
            print(f"[SERVER] Keyboard interrupt received, closing connection with {addr[0]}...")
        finally:
            conn.close()
            print(f"[SERVER] Connection with {addr[0]} closed.")
    conn.close()

# Create server socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(2)  # Listen for two connections

print(f"[SERVER] Listening on port {port}...")

# Prepare CSV files for both master and follower
create_csv(csv_file_master)
create_csv(csv_file_follower)

# Accept client connections
def accept_connections():
    while True:
        conn, addr = server_socket.accept()
        threading.Thread(target=handle_client, args=(conn, addr)).start()

# Start accepting connections
accept_connections()

# Close the server socket upon exit
server_socket.close()
