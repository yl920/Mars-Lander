import numpy as np
import matplotlib.pyplot as plt

results = np.loadtxt('/Assignment 3/trajectories.txt')
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.title('Euler integration')
plt.legend()
plt.show()

results = np.loadtxt('/Assignment 3/verlet_traj.txt')
plt.figure(2)
plt.xlabel('time (s)')
plt.grid()
plt.plot(results[:, 0], results[:, 1], label='x (m)')
plt.plot(results[:, 0], results[:, 2], label='v (m/s)')
plt.title('Verlet integration')
plt.legend()
plt.show()
