#
#  ISC License

#  Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.

#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

#
# Plot Prescribed Gimbal Motion
#
# Author:   Leah Kiner
# Creation Date:  July 26, 2023
#

import os
import numpy as np
import pandas as pd
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from Basilisk.utilities import RigidBodyKinematics as rbk


def run():
    deg2Rad = np.pi / 180

    path_to_file = "/Users/leahkiner/Desktop/sigma_FMData.txt"
    sigma_FMData = np.loadtxt(path_to_file)

    # Orientation desired for initial paper concept figure
    # rad1 = 15 * np.pi / 180
    # rad2 = 15 * np.pi / 180
    # dcm1 = np.array([[1,  0, 0],
    #                  [0, np.cos(rad1), np.sin(rad1)],
    #                  [0, -np.sin(rad1), np.cos(rad1)]])
    # dcm2 = np.array([[np.cos(rad2),  0, -np.sin(rad2)],
    #                  [0, 1, 0],
    #                  [np.sin(rad2),  0, np.cos(rad2)]])
    # dcm_FM = np.matmul(dcm2, dcm1)

    # Initial platform orientation
    dcm_FM = rbk.MRP2C(sigma_FMData[0, :])

    # Final platform orientation
    # dcm_FM = rbk.MRP2C(sigma_FMData.iat[-1, :])

    # Determine gimbal frame basis vectors written in mount frame components
    frameVec1 = dcm_FM[0, :]  # [f_1]
    frameVec2 = dcm_FM[1, :]  # [f_2]
    frameVec3 = dcm_FM[2, :]  # [f_3]

    # Choose platform thickness
    thk = 0.02

    # Create cylindrical hub
    radius = 0.75
    x_center = radius
    y_center = 0.0
    z_height = -4
    z = np.linspace(0, z_height, 25)
    theta = np.linspace(0, 2*np.pi, 25)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + x_center
    y_grid = radius*np.sin(theta_grid) + y_center

    # Create the concept figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the cylinder hub
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.4, linewidth=0.75, color="gray")

    # Create circle for the end of the cylinder hub
    circ = Circle((radius, 0), radius, alpha=0.4, linewidth=0.75, color="gray")

    # Plot the circle
    ax.add_patch(circ)
    art3d.pathpatch_2d_to_3d(circ, z=0.0, zdir="z")

    # Plot the mount frame
    ax.quiver(radius, 0, 0, 1, 0, 0, linewidth=2, color="darkturquoise")
    ax.quiver(radius, 0, 0, 0, 1, 0, linewidth=2, color="mediumslateblue")
    ax.quiver(radius, 0, 0, 0, 0, 1, linewidth=2, color="orchid")
    plt.axis('equal')

    # Create the gimbal platform as a cube
    faces = []
    faces.append(np.zeros([5, 3]))
    faces.append(np.zeros([5, 3]))
    faces.append(np.zeros([5, 3]))
    faces.append(np.zeros([5, 3]))
    faces.append(np.zeros([5, 3]))
    faces.append(np.zeros([5, 3]))
    # Note: Read face vertex coordinates column-wise (5 vertices)
    c = 0.8
    # Bottom face
    faces[0][:, 0] = [c * frameVec1[0], c * frameVec2[0], -c * frameVec1[0], -c * frameVec2[0], c * frameVec1[0]]
    faces[0][:, 1] = [c * frameVec1[1], c * frameVec2[1], -c * frameVec1[1], -c * frameVec2[1], c * frameVec1[1]]
    faces[0][:, 2] = [c * frameVec1[2], c * frameVec2[2], -c * frameVec1[2], -c * frameVec2[2], c * frameVec1[2]]
    # Top face
    faces[1][:, 0] = [(thk * frameVec3[0]) + (c * frameVec1[0]), (thk * frameVec3[0]) + (c * frameVec2[0]), (thk * frameVec3[0]) + (-c * frameVec1[0]), (thk * frameVec3[0]) + (-c * frameVec2[0]), c * frameVec1[0]]
    faces[1][:, 1] = [(thk * frameVec3[1]) + (c * frameVec1[1]), (thk * frameVec3[1]) + (c * frameVec2[1]), (thk * frameVec3[1]) + (-c * frameVec1[1]), (thk * frameVec3[1]) + (-c * frameVec2[1]), c * frameVec1[1]]
    faces[1][:, 2] = [(thk * frameVec3[2]) + (c * frameVec1[2]), (thk * frameVec3[2]) + (c * frameVec2[2]), (thk * frameVec3[2]) + (-c * frameVec1[2]), (thk * frameVec3[2]) + (-c * frameVec2[2]), c * frameVec1[2]]
    # Left Face
    faces[2][:, 0] = [-c * frameVec2[0], c * frameVec1[0], (thk * frameVec3[0]) + (c * frameVec1[0]), (thk * frameVec3[0]) - (c * frameVec2[0]), -c * frameVec2[0]]
    faces[2][:, 1] = [-c * frameVec2[1], c * frameVec1[1], (thk * frameVec3[1]) + (c * frameVec1[1]), (thk * frameVec3[1]) - (c * frameVec2[1]), -c * frameVec2[1]]
    faces[2][:, 2] = [-c * frameVec2[2], c * frameVec1[2], (thk * frameVec3[2]) + (c * frameVec1[2]), (thk * frameVec3[2]) - (c * frameVec2[2]), -c * frameVec2[2]]
    # Right Face
    faces[3][:, 0] = [-c * frameVec1[0], c * frameVec2[0], (thk * frameVec3[0]) + (c * frameVec2[0]), (thk * frameVec3[0]) + (-c * frameVec1[0]), -c * frameVec1[0]]
    faces[3][:, 1] = [-c * frameVec1[1], c * frameVec2[1], (thk * frameVec3[1]) + (c * frameVec2[1]), (thk * frameVec3[1]) + (-c * frameVec1[1]), -c * frameVec1[1]]
    faces[3][:, 2] = [-c * frameVec1[2], c * frameVec2[2], (thk * frameVec3[2]) + (c * frameVec2[2]), (thk * frameVec3[2]) + (-c * frameVec1[2]), -c * frameVec1[2]]
    # Front face
    faces[4][:, 0] = [c * frameVec1[0], c * frameVec2[0], (thk * frameVec3[0]) + (c * frameVec2[0]), (thk * frameVec3[0]) + (c * frameVec1[0]), c * frameVec1[0]]
    faces[4][:, 1] = [c * frameVec1[1], c * frameVec2[1], (thk * frameVec3[1]) + (c * frameVec2[1]), (thk * frameVec3[1]) + (c * frameVec1[1]), c * frameVec1[1]]
    faces[4][:, 2] = [c * frameVec1[2], c * frameVec2[2], (thk * frameVec3[2]) + (c * frameVec2[2]), (thk * frameVec3[2]) + (c * frameVec1[2]), c * frameVec1[2]]
    # Back face
    faces[5][:, 0] = [-c * frameVec2[0], -c * frameVec1[0], (thk * frameVec3[0]) + (-c * frameVec1[0]), (thk * frameVec3[0]) + (-c * frameVec2[0]), -c * frameVec2[0]]
    faces[5][:, 1] = [-c * frameVec2[1], -c * frameVec1[1], (thk * frameVec3[1]) + (-c * frameVec1[1]), (thk * frameVec3[1]) + (-c * frameVec2[1]), -c * frameVec2[1]]
    faces[5][:, 2] = [-c * frameVec2[2], -c * frameVec1[2], (thk * frameVec3[2]) + (-c * frameVec1[2]), (thk * frameVec3[2]) + (-c * frameVec2[2]), -c * frameVec2[2]]

    # Plot the gimbal platform
    ax.add_collection3d(Poly3DCollection(faces, facecolors='mediumseagreen', linewidth=0.2, edgecolors='k', alpha=0.3))

    # Plot the gimbal frame
    ax.quiver(0, 0, 0, c*frameVec1[0], c*frameVec1[1], c*frameVec1[2], linewidth=2, color="darkturquoise")
    ax.quiver(0, 0, 0, c*frameVec2[0], c*frameVec2[1], c*frameVec2[2], linewidth=2, color="mediumslateblue")
    ax.quiver(0, 0, 0, c*frameVec3[0], c*frameVec3[1], c*frameVec3[2], linewidth=2, color="orchid")

    # Turn off the gridlines
    plt.axis('off')
    plt.grid(b=None)

    # Adjust the initial view of the concept figure
    ax.view_init(25, 70, -25)

    plt.show()
    plt.close("all")

if __name__ == "__main__":
    run()
