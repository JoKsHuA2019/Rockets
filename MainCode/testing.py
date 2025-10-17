import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random

plt.ion()

fig, axes = plt.subplots(2, 2, layout='constrained')
(ax1, ax2), (ax3, ax4) = axes
plt.tight_layout()

time_data = []
velocity_data = []

line1, = ax1.plot([], [])
line2, = ax2.plot([], [])

ax1.set_title('Rocket Velocity vs Time')
ax1.set_xlabel('time [s]')
ax1.set_ylabel('velocity [m/s]')

ax2.set_xlabel('time [s]')
ax2.set_ylabel('acceleration [m/s^2]')

ax3.set_xlabel('time [s]')
ax3.set_ylabel('x fin position [degree]')

ax4.set_xlabel('time [s]')
ax4.set_ylabel('y fin position [degree]')

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1, line2

for frame in range(300):
    time = frame * 0.1
    time_data.append(time)
    velocity_data.append(50 + 20*np.sin(time))

    if (time%0.5 == 0):
        line1.set_data(time_data, velocity_data)
        ax1.relim()
        ax1.autoscale_view()

        plt.draw()
        plt.pause(0.01)

