import csv

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

# Example usage:
amp = 0.4  # Amplitude
T = 5  # Duration of the motion (seconds)
middlePoint = 0.69  # Middle point (usually 0)

relativeZ = 0  # Initial position (starting at middle point)
moving_up = True  # Start moving upwards

# Prepare the CSV file to store the values
csv_filename = "motion_data.csv"

# Open the CSV file for writing
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Write the header (optional)
    writer.writerow(["Time (s)", "Z Position"])
    
    # Simulate the motion for 30 seconds with 500 updates per second
    for time_step in range(500 * T):  # Loop for 30 seconds (500 times per second)
        # Calculate the current position
        z, relativeZ, moving_up = getSegmentTargetConstantSpeed(relativeZ, amp, T, middlePoint, moving_up)
        
        # Time in seconds (current time step / 500 calls per second)
        time_in_seconds = time_step / 500
        
        # Write the time and Z position to the CSV file
        writer.writerow([time_in_seconds, z])

print(f"Data saved to {csv_filename}")
