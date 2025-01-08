import rtde_receive
import rtde_control
import time
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.56.101")


# Loop to increment the first joint's position
try:
    while True:
        actual_q = rtde_r.getActualQ()
        print(str(actual_q))
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Program interrupted by user.")