import matplotlib.pyplot as plt
import csv

def plot_tcp_data(file_path):
    # Prepare lists to store data
    timestamps = []
    master_y = []
    master_z = []
    slave_y = []
    slave_z = []

    # Read the CSV file and extract the relevant data
    with open(file_path, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            timestamps.append(float(row[0]))  # Timestamp (s)
            master_y.append(float(row[1]))   # Master TCP Y
            master_z.append(float(row[2]))   # Master TCP Z
            slave_y.append(float(row[3]))    # Slave TCP Y
            slave_z.append(float(row[4]))    # Slave TCP Zzz

    # Create a figure with two subplots (axes)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    # Plot Master vs Slave for Y values
    ax1.plot(timestamps, master_y, label='Master Y', color='blue')
    ax1.plot(timestamps, slave_y, label='Slave Y', color='red')
    ax1.set_title("Master vs Slave TCP Y")
    ax1.set_xlabel("Timestamp (s)")
    ax1.set_ylabel("TCP Y")
    ax1.legend()

    # Plot Master vs Slave for Z values
    ax2.plot(timestamps, master_z, label='Master Z', color='blue')
    ax2.plot(timestamps, slave_z, label='Slave Z', color='red')
    ax2.set_title("Master vs Slave TCP Z")
    ax2.set_xlabel("Timestamp (s)")
    ax2.set_ylabel("TCP Z")
    ax2.legend()

    # Adjust layout for better presentation
    plt.tight_layout()

    # Show the plots
    plt.show()

# Example usage:
plot_tcp_data("./tcp_data.csv")