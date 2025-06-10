import numpy as np
import matplotlib.pyplot as plt

# Motion parameters
aM = 0.07       # Master radius
aP = 0.0425     # Partner radius
wM = 2 * np.pi / 30  # Master angular speed (rad/s)
wP = 2 * np.pi / 5   # Partner angular speed (rad/s)
dt = 0.002      # Timestep (2ms)
T = 30          # Total time in seconds

# Time vector
t = np.arange(0, T, dt)

# Master trajectory (circular)
xM = aM * np.cos(wM * t)
yM = aM * np.sin(wM * t)

# Partner trajectory (circular around moving master)
xP = xM + aP * np.cos(wP * t)
yP = yM + aP * np.sin(wP * t)

# Plotting
plt.figure(figsize=(8, 8))
plt.plot(xM, yM, label='Master Trajectory', linewidth=2)
plt.plot(xP, yP, label='Partner Trajectory', linewidth=2)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Ideal Trajectories of Master and Partner Robots')
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.show()
