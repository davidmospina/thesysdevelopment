from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time

rtde_c = RTDEControl("192.168.56.102")
rtde_r = RTDEReceive("192.168.56.102")


# Target in the Z-Axis of the TCP
target = rtde_r.getActualTCPPose()
print(str(target))

# Stop the RTDE control script
rtde_c.stopScript()
