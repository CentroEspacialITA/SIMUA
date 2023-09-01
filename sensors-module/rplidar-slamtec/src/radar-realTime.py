#!/usr/bin/env python3
'''Animates distances and measurement quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = '/dev/ttyUSB0'
DMAX = 4000
IMIN = 0
IMAX = 100

def update_line(num, iterator, line, ax):
    scan = next(iterator)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)

    min_dist = np.min(intens)
    max_dist = np.max(intens)
    legend_text = f'Min: {min_dist:.2f} Max: {max_dist:.2f}'
    ax.legend([legend_text], loc='lower right')

    # Display angle values in lime
    ax.set_xticklabels(['0°', '45°', '90°', '135°', '180°', '225°', '270°', '315°'], color='lime')

    return line,

def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    fig.patch.set_facecolor('black')  # Set the background color to black

    ax = plt.subplot(111, projection='polar')
    ax.xaxis.grid(True, color='#00CC00', linestyle='dashed')
    ax.yaxis.grid(True, color='#00CC00', linestyle='dashed')

    ax.set_facecolor('black')  # Set the radar grid background color to black
    ax.spines['polar'].set_visible(False)  # Hide the radial lines

    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                        cmap=plt.cm.Reds_r, lw=0)  # Use 'Reds' colormap
    ax.set_rmax(DMAX)
    ax.grid(True, color='lime', linestyle='-')  # Use lime color for the radar grid

    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line,
        fargs=(iterator, line, ax), interval=50)
    ax.tick_params(axis='both', colors='#00CC00')
    plt.legend()  # Show the legend
    plt.show()
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()
