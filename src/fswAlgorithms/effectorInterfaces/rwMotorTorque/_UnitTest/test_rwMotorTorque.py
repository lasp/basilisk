# SPDX-License-Identifier: ISC
# Copyright (c) 2016, Autonomous Vehicle System Lab, University of Colorado at Boulder
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#

"""
Module Name:        rwMotorTorque
"""

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import all of the modules that we are going to be called in this simulation
from xmera.utilities import SimulationBaseClass
from xmera.fswAlgorithms import rwMotorTorque
from xmera.utilities import macros
from xmera.architecture import messaging

@pytest.mark.parametrize("num_control_axes", [0, 1, 2, 3])
@pytest.mark.parametrize("num_wheels", [2, 4, messaging.RW_EFF_CNT])
@pytest.mark.parametrize("num_input_cmd_torques", [1, 2])
@pytest.mark.parametrize("rw_avail_msg",["NO", "ON", "OFF", "MIXED"])

def test_rw_motor_torque(show_plots, num_control_axes, num_wheels, num_input_cmd_torques, rw_avail_msg):
    # @TODO With the current implementation of throwing an exception when zero control axes are specified, Python quits
    #  and causes all unit tests to fail. Until a different way of handling exceptions or errors is implemented, the
    #  test with 0 control axes is skipped.
    if num_control_axes == 0:
        pytest.skip("Zero control axes can currently not be tested.")

    # In case compile-time max RW number is less than parametrized number of wheels, skip test
    if num_wheels > messaging.RW_EFF_CNT:
        pytest.skip("Number of reaction wheels greater than compile time RW_EFF_CNT.")

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(0.5)     # update process rate update time
    test_proc = unit_test_sim.CreateNewProcess(unit_process_name)
    test_proc.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    module = rwMotorTorque.RwMotorTorque()
    module.modelTag = "rwMotorTorque"

    # Initialize module variables
    if num_control_axes == 3:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    elif num_control_axes == 2:
        control_axes_B = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    elif num_control_axes == 1:
        control_axes_B = [[1, 0, 0], [0, 0, 0], [0, 0, 0]]
    else:
        control_axes_B = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

    module.controlAxes_B = control_axes_B

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module)

    torque = np.array([1.0, -0.5, 0.7])

    # attControl message
    input_message_data = messaging.CmdTorqueBodyMsgPayload()
    requested_torque1 = torque
    input_message_data.torqueRequestBody = requested_torque1
    cmd_torque_in_msg = messaging.CmdTorqueBodyMsg().write(input_message_data)

    requested_torque = np.array(requested_torque1)

    if num_input_cmd_torques == 2:
        input_message_data2 = messaging.CmdTorqueBodyMsgPayload()
        requested_torque2 = np.array([[1.1, -1.3, 2.0], [0.3, 0.9, -1.4], [2.2, 1.7, 0.6]]) @ torque
        input_message_data2.torqueRequestBody = requested_torque2
        cmd_torque_in2_msg = messaging.CmdTorqueBodyMsg().write(input_message_data2)
        requested_torque += np.array(requested_torque2)

    # wheelConfigData message
    rw_config_params = messaging.RWArrayConfigMsgPayload()
    RW_EFF_CNT = messaging.RW_EFF_CNT

    if num_wheels == 4:
        rw_config_params.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]
    else:
        # create some arbitrary spin axes
        spin_axis = np.array([0.56, -1.73, 0.22])
        G_s_B = []
        for rw in range(num_wheels):
            spin_axis = np.array([[1.5, -3.7, 0.4], [-0.1, 0.3, -1.9], [3.2, 0.3, 0.8]]) @ spin_axis
            G_s_B.append(spin_axis / np.linalg.norm(spin_axis))

        rw_config_params.GsMatrix_B = np.array(G_s_B).flatten()

    rw_config_params.JsList = [0.1] * num_wheels
    rw_config_params.numRW = num_wheels
    rw_config_in_msg = messaging.RWArrayConfigMsg().write(rw_config_params)

    if rw_avail_msg != "NO":
        rw_availability_message = messaging.RWAvailabilityMsgPayload()
        if rw_avail_msg == "ON":
            avail = [messaging.AVAILABLE] * num_wheels
        elif rw_avail_msg == "OFF":
            avail = [messaging.UNAVAILABLE] * num_wheels
        else:
            avail_alternating = [messaging.AVAILABLE, messaging.AVAILABLE, messaging.UNAVAILABLE] * RW_EFF_CNT
            avail = avail_alternating[:num_wheels]

        rw_availability_message.wheelAvailability = avail

        rw_avail_in_msg = messaging.RWAvailabilityMsg().write(rw_availability_message)
        module.rwAvailInMsg.subscribeTo(rw_avail_in_msg)
    else:
        avail = [rwMotorTorque.AVAILABLE] * num_wheels  # this is used purely for the python level solution

    # Setup logging on the test module output message so that we get all the writes to it
    data_log = module.rwMotorTorqueOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, data_log)

    # connect messages
    module.vehControlInMsg.subscribeTo(cmd_torque_in_msg)
    if num_input_cmd_torques == 2:
        module.vehControlIn2Msg.subscribeTo(cmd_torque_in2_msg)
    module.rwParamsInMsg.subscribeTo(rw_config_in_msg)

    unit_test_sim.InitializeSimulation()
    module.reset(0)

    # Set the simulation time.
    unit_test_sim.ConfigureStopTime(macros.sec2nano(0.5))

    # Begin the simulation time run set above
    unit_test_sim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    motor_torque = data_log.motorTorque

    # set the output truth states
    u_s = compute_true_torque(np.array(control_axes_B),
                              np.array(rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F'),
                              requested_torque,
                              avail)

    true_motor_torque = [u_s] * 2

    # compare the module results to the truth values
    accuracy = 1e-8
    np.testing.assert_allclose(motor_torque, true_motor_torque, rtol=0, atol=accuracy, verbose=True)

    G_s_B =np.array( rw_config_params.GsMatrix_B).reshape((3, RW_EFF_CNT), order='F')
    F = np.transpose(motor_torque[0])
    received_torque = -(G_s_B @ F).flatten()

    if num_wheels >= num_control_axes > 0:
        if (len(avail) - np.sum(avail)) > num_control_axes:
            np.testing.assert_allclose(received_torque[:num_control_axes], requested_torque[:num_control_axes],
                                       rtol=0,
                                       atol=accuracy,
                                       verbose=True)


def compute_true_torque(C, Gs_B, Lr, avail_msg):

    num_control_axes = (np.linalg.norm(C, axis=1) > 0.0).sum()
    num_wheels = len(avail_msg)
    non_avail_wheels = 0

    # Remove wheels that are deemed unavailable
    for i in range(len(Gs_B[0])): #
        if num_wheels > i:
            if avail_msg[i] != messaging.AVAILABLE:
                Gs_B[:,i] = [0.0, 0.0, 0.0]
                non_avail_wheels += 1
        else:
            Gs_B[:,i] = [0.0, 0.0, 0.0]

    # If fewer wheels than number of control axes, output no torque
    if (num_wheels-non_avail_wheels) < num_control_axes:
        return [0.0]*len(Gs_B[0])

    Lr_C = np.dot(C,Lr) # Project torque onto control axes
    CGs = np.dot(C, Gs_B) # Map the control axes onto the wheels

    # Build minimum norm framework
    M = np.dot(CGs, CGs.T)
    M_rep = np.identity(3) # Need to keep the matrix non-singular for inversion
    for i in range(0,num_control_axes):
        for j in range(0,num_control_axes):
            M_rep[i][j] = M[i][j]
    M_inv = np.linalg.inv(M_rep)

    # Remove projection to any non-defined control axes
    for i in range(num_control_axes,3):
        M_inv[i][i] = 0.0

    # Determine the solution
    L = np.dot(M_inv, Lr_C)

    # Map the solution to the wheels
    u_s = np.dot(CGs.T, L)

    return -u_s


if __name__ == "__main__":
    test_rw_motor_torque(False,
                         3,  # numControlAxes
                         messaging.RW_EFF_CNT,  # numWheels
                         2,  # numInputCmdTorques
                         "NO"  # RWAvailMsg ("NO", "ON", "OFF")
                         )
