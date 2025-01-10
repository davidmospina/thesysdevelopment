import rtde_receive
import rtde_control
import time
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.101")
rtde_c = rtde_control.RTDEControlInterface("192.168.56.102")
velocity = 0.5
acceleration = 0.5
dt = 1.0/500  # 2ms
lookahead_time = 0.1
gain = 300

# Loop to increment the first joint's position
try:
    while True:
        t_start = rtde_c.initPeriod()
        actual_q = rtde_r.getActualQ()
        rtde_c.servoJ(actual_q, velocity, acceleration, dt, lookahead_time, gain)
        rtde_c.waitPeriod(t_start)
except KeyboardInterrupt:
    print("Program interrupted by user.")