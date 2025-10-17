import matplotlib.pyplot as plt
import random

fig, ax = plt.subplots(2, 2, layout='constrained')

graph1_x = [0]

ax[0, 0].set_title('Rocket Velocity vs Time')
ax[0, 0].set_xlabel('time [s]')
ax[0, 0].set_ylabel('velocity [m/s]')

ax[0, 1].set_xlabel('time [s]')
ax[0, 1].set_ylabel('acceleration [m/s^2]')

ax[1, 0].set_xlabel('time [s]')
ax[1, 0].set_ylabel('x fin position [degree]')

ax[1, 1].set_xlabel('time [s]')
ax[1, 1].set_ylabel('y fin position [degree]')

while True:
    ax[0, 0].set_xlim()

plt.show()