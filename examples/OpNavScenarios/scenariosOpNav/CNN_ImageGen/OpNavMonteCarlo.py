#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
r"""
Overview
--------

The OpNav Monte-Carlo python scripts provides the capability to generate images and truth data in order to
train neural networks for image processing.

This script calls OpNavScenarios/CNN_ImageGen/scenario_CNNImages.py in order to generate the simulations.
The script can be called by running::

    python3 OpNavMonteCarlo.py

"""



import csv
import inspect
import os

import scenario_CNNImages as scenario

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk import __path__
bskPath = __path__[0]

from Basilisk.utilities.MonteCarlo.Controller import Controller, RetentionPolicy
from Basilisk.utilities.MonteCarlo.Dispersions import OrbitalElementDispersion, MRPDispersionPerAxis, UniformDispersion

# import simulation related support
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
import matplotlib.pyplot as plt
import numpy as np


retainedMessageName1 = "scMsg"
retainedMessageName2 = "circlesMsg"
retainedMessageName3 = "cob_msg"
retainedRate = macros.sec2nano(10)
var1 = "r_BN_N"
var2 = "sigma_BN"
var3 = "valid"
var4 = "centerOfBrightness"

def run(show_plots):
    """Main Simulation Method"""

    NUMBER_OF_RUNS = 122
    PROCESSES = 1
    RUN = False
    POST = True

    dirName = os.path.abspath(os.path.dirname(__file__)) + "/cnn_MC_data"
    if RUN:
        myExecutionFunction = scenario.run
        myCreationFunction = scenario.scenario_OpNav

        monteCarlo = Controller()
        monteCarlo.setShouldDisperseSeeds(True)
        monteCarlo.setExecutionFunction(myExecutionFunction)
        monteCarlo.setSimulationFunction(myCreationFunction)
        monteCarlo.setExecutionCount(NUMBER_OF_RUNS)
        monteCarlo.setThreadCount(PROCESSES)
        monteCarlo.setVerbose(False)
        monteCarlo.setArchiveDir(dirName)

        # Add some dispersions
        dispDict = {}
        dispDict["mu"] = 1.9
        dispDict["a"] = ["uniform", 50*1E3, 100*1E3] # 12000
        dispDict["e"] = ["uniform", 0.2, 0.5]           # 0.4, 0.7
        dispDict["i"] = ["uniform", np.deg2rad(40), np.deg2rad(90)]
        dispDict["Omega"] = None
        dispDict["omega"] = None
        dispDict["f"] = ["uniform", np.deg2rad(0), np.deg2rad(359)]

        disp1Name = 'get_DynModel().scObject.hub.r_CN_NInit'
        disp2Name = 'get_DynModel().scObject.hub.v_CN_NInit'
        disp3Name = 'get_FswModel().trackingErrorCam.sigma_R0R'

        monteCarlo.addDispersion(OrbitalElementDispersion(disp1Name,disp2Name, dispDict))
        monteCarlo.addDispersion(MRPDispersionPerAxis(disp3Name, bounds=[[1./3-0.051, 1./3+0.051], [1./3-0.051, 1./3+0.051], [-1./3-0.051, -1./3+0.051]]))

        # Add retention policy
        retentionPolicy = RetentionPolicy()
        retentionPolicy.addMessageLog(retainedMessageName1, [var1, var2])
        retentionPolicy.addMessageLog(retainedMessageName2, [var3])
        retentionPolicy.addMessageLog(retainedMessageName3, [var3, var4])
        monteCarlo.addRetentionPolicy(retentionPolicy)

        failures = monteCarlo.executeSimulations()
        assert len(failures) == 0, "No runs should fail"

    if POST:
        monteCarlo = Controller.load(dirName)
        for i in range(0,NUMBER_OF_RUNS):
            try:
                monteCarloData = monteCarlo.getRetainedData(i)
            except FileNotFoundError:
                print("File not found, ",  i)
                continue
            csvfile = open(dirName + "/run" + str(i) + "/data.csv", 'w')
            writer = csv.writer(csvfile)
            writer.writerow(['Filename', 'Valid', 'X_com', 'Y_com', 'X_cob', 'Y_cob'])

            position_N =monteCarloData["messages"][retainedMessageName1 + "." + var1]
            sigma_BN =monteCarloData["messages"][retainedMessageName1 + "." + var2]
            validCircle =monteCarloData["messages"][retainedMessageName3 + "." + var3]
            cob =monteCarloData["messages"][retainedMessageName3 + "." + var4]

            renderRate = 60*1E9
            sigma_CB = [0., 0., 0.]  # Arbitrary camera orientation
            sizeOfCam = [512, 512]
            sizeMM = [10. * 1E-3, 10. * 1E-3]  # in m
            fieldOfView = np.deg2rad(55)  # in degrees
            focal = sizeMM[0] / 2. / np.tan(fieldOfView / 2.)  # in m

            pixelSize = []
            pixelSize.append(sizeMM[0] / sizeOfCam[0])
            pixelSize.append(sizeMM[1] / sizeOfCam[1])

            # camera parameters
            alpha = 0.
            pX = 2.*np.tan(fieldOfView*sizeOfCam[0]/sizeOfCam[1]/2.0)
            pY = 2.*np.tan(fieldOfView/2.0)
            dX = focal/pixelSize[0]
            dY = focal/pixelSize[1]
            up = sizeOfCam[0]/2.
            vp = sizeOfCam[1]/2.
            # build camera calibration matrix (not the inverse of it)
            K = np.array([[dX, alpha, up], [0., dY, vp], [0., 0., 1.]])

            dcm_CB = rbk.MRP2C(sigma_CB)
            # Plot results
            for i in range(len(validCircle[:, 0])):
                if validCircle[i, 1] > 0:
                    r_image_plane_N = position_N[i,1:]
                    dcm_BN = rbk.MRP2C(sigma_BN[i,1:])
                    r_image_plane_C = np.dot(dcm_CB, np.dot(dcm_BN, r_image_plane_N))
                    com = np.dot(K, r_image_plane_C/r_image_plane_C[2])

                    writer.writerow([str("{0:.6f}".format(position_N[i,0]*1E-9))+".jpg", validCircle[i, 1], com[0], com[1], cob[i, 1], cob[i,2]])
            csvfile.close()

    if show_plots:
        monteCarlo.executeCallbacks()
        plt.show()

    return


if __name__ == "__main__":
    pid = run(True)
