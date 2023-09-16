import numpy as np
import matplotlib.pyplot as plt
results = np.loadtxt("C:/Engineering/Mars-Lander/Mars-Lander/lander/autopilot.txt")
plt.figure(1)
plt.clf()
plt.xlabel('Altitude (m)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='Target descent rate (m/s)')
plt.plot(results[:, 0], results[:, 2], label='Actual descent rate (m/s)')
plt.legend()
plt.show()