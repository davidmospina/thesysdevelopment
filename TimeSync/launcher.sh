#!/bin/bash

# --- Configuration ---
# ROBOT 1
IP1="10.0.0.3" 
USER1="robot1_username" # IMPORTANT: Replace with the actual username. For UR robots "root" is the default

# Device 2
IP2="10.0.0.2"
USER2="robot2_username" 

# --- Commands to be executed remotely ---
PTP4L_COMMAND="/usr/sbin/ptp4l -m -f /etc/ptp4l.conf"
PHC2SYS_COMMAND="/usr/sbin/phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -O 1 -m"

# --- Functions ---
run_remote_commands() {
  local ip="$1"
  local user="$2"
  local device_label="$3"

  echo "--- Connecting to $device_label ($ip) ---"

  # Send ptp4l command
  ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$user@$ip" \
    "nohup sudo $PTP4L_COMMAND > /tmp/ptp4l.log 2>&1 &"

  if [ $? -eq 0 ]; then
    echo "  ptp4l launch command sent to $device_label."
  else
    echo "  Failed to send ptp4l command to $device_label."
    return 1 # Indicate failure
  fi

  echo "  Waiting a few seconds for ptp4l on $device_label to initialize..."
  sleep 5

  # Send phc2sys command
  ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 "$user@$ip" \
    "nohup sudo $PHC2SYS_COMMAND > /tmp/phc2sys.log 2>&1 &"

  if [ $? -eq 0 ]; then
    echo "  phc2sys launch command sent to $device_label."
  else
    echo "  Failed to send phc2sys command to $device_label."
    return 1 # Indicate failure
  fi

  echo "--- Finished with $device_label ---"
  echo #blank line
}

# --- Main Script ---
echo "Starting PTP synchronization process..."
echo

run_remote_commands "$IP1" "$USER1" "Device 1"
echo

run_remote_commands "$IP2" "$USER2" "Device 2"

echo
echo "--- Sync method activation process initiated on both devices. ---"
echo "Check /tmp/ptp4l.log and /tmp/phc2sys.log on each remote device for status and errors."

exit 0
