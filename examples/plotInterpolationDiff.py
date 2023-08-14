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
from matplotlib import collections as mc



def run():
    deg2Rad = np.pi / 180

    # Load gimbal tip & tilt data
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tip_dataM1M2.txt"
    gimbalTipAngleDataM1M2 = np.loadtxt(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tilt_dataM1M2.txt"
    gimbalTiltAngleDataM1M2 = np.loadtxt(path_to_file)

    path_to_file = "/Users/leahkiner/Desktop/gimbal_tip_dataM2M1.txt"
    gimbalTipAngleDataM2M1 = np.loadtxt(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tilt_dataM2M1.txt"
    gimbalTiltAngleDataM2M1 = np.loadtxt(path_to_file)

    path_to_file = "/Users/leahkiner/Desktop/gimbal_tip_dataM1M2_4.txt"
    gimbalTipAngleDataM1M2_4 = np.loadtxt(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tilt_dataM1M2_4.txt"
    gimbalTiltAngleDataM1M2_4 = np.loadtxt(path_to_file)

    path_to_file = "/Users/leahkiner/Desktop/gimbal_tip_dataM2M1_8.txt"
    gimbalTipAngleDataM2M1_8 = np.loadtxt(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tilt_dataM2M1_8.txt"
    gimbalTiltAngleDataM2M1_8 = np.loadtxt(path_to_file)

    path_to_file = "/Users/leahkiner/Desktop/gimbal_tip_dataM1M2_16.txt"
    gimbalTipAngleDataM1M2_16 = np.loadtxt(path_to_file)
    path_to_file = "/Users/leahkiner/Desktop/gimbal_tilt_dataM1M2_16.txt"
    gimbalTiltAngleDataM1M2_16 = np.loadtxt(path_to_file)

    # Plot 2D Diamond and the corresponding gimbal trajectory
    scaleG1 = 25 / 18.342
    scaleG2 = 30 / 27.43
    scaleMotor = 128 / 160
    line1 = [(18.342 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, 27.43 * scaleG2)]
    line2 = [(-18.764 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, 27.43 * scaleG2)]
    line3 = [(-18.764 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, -27.43 * scaleG2)]
    line4 = [(18.342 * scaleG1, 0.0 * scaleG2), (0.676 * scaleG1, -27.43 * scaleG2)]
    lines = [line1, line2, line3, line4]
    lc = mc.LineCollection(lines, colors="crimson", linewidths=2, linestyles="dashed")
    plt.figure()
    plt.clf()
    ax = plt.axes()
    ax.add_collection(lc)
    plt.plot(gimbalTipAngleDataM1M2, gimbalTiltAngleDataM1M2, linewidth=2, color='mediumseagreen', label="M1,M2 256")
    plt.plot(gimbalTipAngleDataM2M1, gimbalTiltAngleDataM2M1, linewidth=2, color='cyan', label="M2,M1 256")
    plt.plot(gimbalTipAngleDataM1M2_4, gimbalTiltAngleDataM1M2_4, linewidth=2, color='mediumslateblue', label="M1,M2 128")
    plt.plot(gimbalTipAngleDataM2M1_8, gimbalTiltAngleDataM2M1_8, linewidth=2, color='orchid', label="M2,M1 64")
    plt.plot(gimbalTipAngleDataM1M2_16, gimbalTiltAngleDataM1M2_16, linewidth=2, color='darkturquoise', label="M1,M2 32")
    ax.scatter(gimbalTipAngleDataM1M2[0], gimbalTiltAngleDataM1M2[0], marker='.', linewidth=5, color='springgreen', label='Initial')
    ax.scatter(gimbalTipAngleDataM1M2[-1], gimbalTiltAngleDataM1M2[-1], marker='*', linewidth=5, color='magenta', label='Final')
    # plt.title('Gimbal Sequential Trajectory', fontsize=14)
    plt.ylabel(r'$\psi$ (deg)', fontsize=16)
    plt.xlabel(r'$\phi$ (deg)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)
    plt.axis("equal")

    plt.show()
    plt.close("all")

if __name__ == "__main__":
    run()