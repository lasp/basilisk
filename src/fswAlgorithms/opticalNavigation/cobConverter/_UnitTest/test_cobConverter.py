#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import inspect
import numpy as np
import os
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import cobConverter
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

noCorr = cobConverter.PhaseAngleCorrectionMethod_NoCorrection
binary = cobConverter.PhaseAngleCorrectionMethod_Binary
lambertian = cobConverter.PhaseAngleCorrectionMethod_Lambertian


def mapState(state, input_camera):
    """Main method that is writen in the module to map the state from pixel space to position"""
    K = compute_camera_calibration_matrix(input_camera)
    Kinv = np.linalg.inv(K)

    rHat_BN_C = Kinv @ np.array([state[0], state[1], 1])
    norm_COB_vector = np.linalg.norm(rHat_BN_C)
    rHat_BN_C = -rHat_BN_C / np.linalg.norm(rHat_BN_C)

    return rHat_BN_C, norm_COB_vector


def mapCovar(pixels, input_camera, norm_COB_vector):
    """Secondary method to map the covariance in pixel space to position"""
    K = compute_camera_calibration_matrix(input_camera)
    d_x = K[0, 0]
    d_y = K[1, 1]
    X = 1 / d_x
    Y = 1 / d_y

    scale_factor = np.sqrt(pixels / (4 * np.pi)) / (norm_COB_vector ** 2)

    covar = np.zeros([3, 3])
    covar[0, 0] = X ** 2
    covar[1, 1] = Y ** 2
    covar[2, 2] = 1

    return scale_factor * covar


def compute_camera_calibration_matrix(input_camera):
    """Secondary method to compute camera calibration matrix"""
    # camera parameters
    alpha = 0.
    resX = input_camera.resolution[0]
    resY = input_camera.resolution[1]
    pX = 2. * np.tan(input_camera.fieldOfView / 2.0)
    pY = 2. * np.tan(input_camera.fieldOfView * resY / resX / 2.0)
    dX = resX / pX
    dY = resY / pY
    up = resX / 2.
    vp = resY / 2.
    # build camera calibration matrix
    K = np.array([[dX, alpha, up], [0., dY, vp], [0., 0., 1.]])

    return K


def phase_angle_correction(alpha, method):
    """Secondary method to compute the phase angle correction for COB/COM offset"""
    if method == binary:
        gamma = 4 / (3 * np.pi) * (1 - np.cos(alpha))
    elif method == lambertian:
        gamma = 3 * np.pi / 16 * ((np.cos(alpha) + 1.0) * np.sin(alpha)) / \
                (np.sin(alpha) + (np.pi - alpha) * np.cos(alpha))
    else:
        gamma = 0.0

    return gamma


@pytest.mark.parametrize("method", [noCorr, binary, lambertian])
@pytest.mark.parametrize("distance", [500e3, 5000e3, 50000e3])
@pytest.mark.parametrize("cameraResolution, centerOfBrightness, numberOfPixels, sunDirection",
                         [([512, 512], [152, 251], 75, [-1., -1., 0.]),
                          ([128, 345], [120, 251], 100, [0., -1., 0.]),
                          ([742, 512], [0, 0], 0, [0., 1., 0.]),
                          ([2048, 2048], [1021, 1891], 1000, [0., 1., 1.]),
                          ([1024, 1024], [521, 891], 800, [1., 0., 0.]),
                          ([875, 987], [321, 191], 375, [1., 0.5, 0.3])
                          ])
def test_cob_converter(show_plots, cameraResolution, centerOfBrightness, numberOfPixels,
                       sunDirection, distance,  method):
    cob_converter_test_function(show_plots, cameraResolution, centerOfBrightness, numberOfPixels,
                                sunDirection, distance, method)


def cob_converter_test_function(show_plots, cameraResolution, centerOfBrightness, numberOfPixels,
                                sunDirection, distance, method):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))
    R_object = 25. * 1e3
    att_sigma = 0.001
    covar_att_B = np.diag([att_sigma**2, (0.9*att_sigma)**2, (0.95*att_sigma)**2])
    module = cobConverter.CobConverter(method, R_object)
    module.setAttitudeCovariance(covar_att_B)
    module.setNumStandardDeviations(3)
    module.setStandardDeviation(100)
    module.enableOutlierDetection()
    unitTestSim.AddModelToTask(unitTaskName, module, module)

    r_BdyZero_N = np.array([-distance, -300. * 1e3, 0.])
    v_BdyZero_N = np.array([8. * 1e3, 0., 0.])

    # compute spacecraft pointing
    h_1 = np.array(r_BdyZero_N) / np.linalg.norm(r_BdyZero_N)
    h_3 = np.cross(h_1, np.array(v_BdyZero_N) / np.linalg.norm(v_BdyZero_N))
    h_3 *= 1 / np.linalg.norm(h_3)
    h_2 = np.cross(h_3, h_1) / np.linalg.norm(np.cross(h_3, h_1))
    dcm_BN = np.array([h_1, h_2, h_3])
    sigma_BN = rbk.C2MRP(dcm_BN)

    # set up camera orientation such that it is pointing at the target
    dcm_CB = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])
    sigma_CB = rbk.C2MRP(dcm_CB)

    # Create the input messages.
    inputCamera = messaging.CameraConfigMsgPayload()
    inputCob = messaging.OpNavCOBMsgPayload()
    inputFilter = messaging.FilterMsgPayload()
    inputAtt = messaging.NavAttMsgPayload()
    inputEphem = messaging.EphemerisMsgPayload()

    # Set camera parameters
    inputCamera.fieldOfView = np.deg2rad(20.0)
    inputCamera.resolution = cameraResolution
    inputCamera.sigma_CB = sigma_CB
    inputCamera.ppFocalLength = 0.10
    camInMsg = messaging.CameraConfigMsg().write(inputCamera)
    module.cameraConfigInMsg.subscribeTo(camInMsg)

    # Set center of brightness
    inputCob.centerOfBrightness = centerOfBrightness
    inputCob.pixelsFound = numberOfPixels
    inputCob.timeTag = 12345
    inputCob.valid = (numberOfPixels > 0)
    cobInMsg = messaging.OpNavCOBMsg().write(inputCob)
    module.opnavCOBInMsg.subscribeTo(cobInMsg)

    # Set filter message
    inputFilter.numberOfStates = 6
    inputFilter.state = np.array([r_BdyZero_N, v_BdyZero_N]).flatten()
    inputFilter.covar = np.diag([50e3, 50e3, 50e3, 0.01, 0.01, 0.01]).flatten()
    filterInMsg = messaging.FilterMsg().write(inputFilter)
    module.opnavFilterInMsg.subscribeTo(filterInMsg)

    vehSunPntN = np.array(sunDirection) / np.linalg.norm(np.array(sunDirection))  # unit vector from SC to Sun
    # Set body attitude relative to inertial
    inputAtt.sigma_BN = sigma_BN
    inputAtt.vehSunPntBdy = dcm_BN @ vehSunPntN
    attInMsg = messaging.NavAttMsg().write(inputAtt)
    module.navAttInMsg.subscribeTo(attInMsg)

    # Set ephemeris message of spacecraft relative to object
    inputEphem.r_BdyZero_N = r_BdyZero_N
    ephemInMsg = messaging.EphemerisMsg().write(inputEphem)
    module.ephemInMsg.subscribeTo(ephemInMsg)

    dataLogUnitVecCOB = module.opnavUnitVecCOBOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogUnitVecCOB)
    dataLogUnitVecCOM = module.opnavUnitVecCOMOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogUnitVecCOM)
    dataLogCOM = module.opnavCOMOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogCOM)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    # the target is at the center of the image, as this test assumes perfect pointing at the target.
    # Thus, for the outlier check, the expected COB is at the center of the image.
    # This compares the input COB with the expected COB
    cobCenter = np.array(cameraResolution) / 2
    cobErrorCenter = np.linalg.norm(np.array(centerOfBrightness) - cobCenter)
    acceptedCobError = module.getNumStandardDeviations() * module.getStandardDeviation()

    # Truth Values
    if numberOfPixels > 0:
        goodPixels = 1
    else:
        goodPixels = 0
    cob_true = [inputCob.centerOfBrightness[0], inputCob.centerOfBrightness[1]]
    num_pixels = inputCob.pixelsFound
    dcm_CB = rbk.MRP2C(inputCamera.sigma_CB)
    dcm_BN = rbk.MRP2C(inputAtt.sigma_BN)
    dcm_NC = np.dot(dcm_CB, dcm_BN).T

    # Center of Brightness Unit Vector
    [rhat_COB_C_true, norm_COB_vector] = mapState(cob_true, inputCamera)
    covar_COB_C_true = mapCovar(num_pixels, inputCamera, norm_COB_vector)
    rhat_COB_N_true = np.dot(dcm_NC, rhat_COB_C_true) * goodPixels  # multiple by validity to get zero vector if bad
    timeTag_true_ns = inputCob.timeTag * goodPixels
    timeTag_true = timeTag_true_ns * macros.NANO2SEC

    covar_COB_B_true = np.dot(dcm_CB.T, np.dot(covar_COB_C_true, dcm_CB))
    covar_B_true = covar_COB_B_true + covar_att_B
    covar_N_true = np.dot(dcm_BN.T, np.dot(covar_B_true, dcm_BN)).flatten() * goodPixels

    if goodPixels and cobErrorCenter < acceptedCobError:
        valid_COB_true = True
    else:
        valid_COB_true = False

    # Center of Mass Message and Unit Vector
    alpha = np.arccos(np.dot(r_BdyZero_N.T / np.linalg.norm(r_BdyZero_N), vehSunPntN))  # phase angle
    gamma = phase_angle_correction(alpha, method)  # COB/COM offset factor
    shat_C = dcm_NC.T @ vehSunPntN
    phi = np.arctan2(shat_C[1], shat_C[0])  # sun direction in image plane
    K = compute_camera_calibration_matrix(inputCamera)
    dX = K[0, 0]
    Kx = dX / inputCamera.ppFocalLength
    Rc = R_object * Kx * inputCamera.ppFocalLength / np.linalg.norm(r_BdyZero_N)  # object radius in pixels
    com_true = [None] * 2  # COM location in image
    com_true[0] = cob_true[0] - gamma * Rc * np.cos(phi) * goodPixels
    com_true[1] = cob_true[1] - gamma * Rc * np.sin(phi) * goodPixels
    [rhat_COM_C_true, norm_COM_vector] = mapState(com_true, inputCamera)
    if goodPixels and (method == binary or method == lambertian):
        valid_COM_true = True
        rhat_COM_N_true = np.dot(dcm_NC, rhat_COM_C_true)
    else:
        valid_COM_true = False
        rhat_COM_N_true = rhat_COB_N_true

    # module output
    rhat_COB_N = dataLogUnitVecCOB.rhat_BN_N[0]
    covar_N = dataLogUnitVecCOB.covar_N[0]
    time_COB = dataLogUnitVecCOB.timeTag[0]
    valid_COB = dataLogUnitVecCOB.valid[0]
    com = dataLogCOM.centerOfMass[0]
    time_COM_ns = dataLogCOM.timeTag[0]
    valid_COM = dataLogCOM.valid[0]
    rhat_COM_N = dataLogUnitVecCOM.rhat_BN_N[0]

    # make sure module output data is correct
    tolerance = 1e-10
    np.testing.assert_allclose(rhat_COB_N,
                               rhat_COB_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: rhat_COB_N',
                               verbose=True)

    np.testing.assert_allclose(covar_N,
                               covar_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: covar_N',
                               verbose=True)

    np.testing.assert_allclose(time_COB,
                               timeTag_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: time_COB',
                               verbose=True)
    np.testing.assert_equal(valid_COB,
                            valid_COB_true,
                            err_msg='Variable: valid_COB',
                            verbose=True)
    np.testing.assert_allclose(com,
                               com_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: com',
                               verbose=True)
    np.testing.assert_allclose(time_COM_ns,
                               timeTag_true_ns,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: time_COM_ns',
                               verbose=True)
    np.testing.assert_equal(valid_COM,
                            valid_COM_true,
                            err_msg='Variable: valid_COM',
                            verbose=True)
    np.testing.assert_allclose(rhat_COM_N,
                               rhat_COM_N_true,
                               rtol=0,
                               atol=tolerance,
                               err_msg='Variable: rhat_COM_N',
                               verbose=True)


if __name__ == '__main__':
    test_cob_converter(False, [512, 512], [152, 251], 75, [-1.0, -1.0, 0.0], 36e6, binary)
