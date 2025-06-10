import rtde_receive
import rtde_control
import time
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.102")
rtde_c = rtde_control.RTDEControlInterface("192.168.56.102")

targetA = [0.28697383813837185, -0.14998050513491248, 0.7509854580323809, -0.16554934951083813, 1.567441645128796, -0.2602440801528517]
# targetA = [value / 1000 for value in targetA]        
targetB = targetA.copy()
targetB[2] = 0.300  # Modify the 4th element of targetB
rtde_c.moveJ_IK(targetA, 0.1, 0.2, False)

try:
    while True:
        rtde_c.moveL(targetA, 0.25, 0.5, False) # ACC and VEL and (async = false, does block code until move is finished) 
        rtde_c.moveL(targetB, 0.25, 0.5, False) 

except KeyboardInterrupt:
    print("Program interrupted by user.")

# [0.28697383813837185, -0.14998050513491248, 0.5909854580323809, -0.16554934951083813, 1.567441645128796, -0.2602440801528517]
# #pose = [0.2906405611856461, -0.13341228560357027, 0.8068726896841272, -1.3318598842652503, 1.3331279982685684, -1.1255991733782869]

