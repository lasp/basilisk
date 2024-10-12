#
#  ISC License
#
#  Copyright (c) 2024 University of Colorado at Boulder
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
#
#   Unit Test Script
#   Module Name:        flybyPoint
#   Author:             Riccardo Calaon
#   Creation Date:      May 26, 2023
#

import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk import __path__
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import flybyPoint
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


@pytest.mark.parametrize("initial_position", [[-5e7, 7.5e6, 5e5]])  # m - r_CN_N
@pytest.mark.parametrize("initial_velocity", [[2e4, 0, 0]])  # m/s - v_CN_N
@pytest.mark.parametrize("filter_dt", [1, 60])  # s
@pytest.mark.parametrize("orbit_normal_sign", [1, -1])
@pytest.mark.parametrize("max_rate", [0, 0.01])
@pytest.mark.parametrize("max_acceleration", [0, 1E-7])
def test_flybyPoint(show_plots, initial_position, initial_velocity, filter_dt, orbit_normal_sign, max_rate,
                    max_acceleration):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the reference attitude computed by :ref:`flybyPoint` in a scenario
    where the rectilinear flyby assumption is valid.

    **Test Parameters**

    In this test, there is no gravity body, and the spacecraft is put onto a rectilinear trajectory about the origin.
    With no gravity, the linear momentum of the spacecraft does not change, which means that the spacecraft proceeds
    along a rectilinear trajectory. The input message to the :ref:`flybyPoint` is the relative position and velocity
    of the spacecraft with respect to the body/asteroid, which coincides with the origin and is assumed to be static.
    Correctness is tested assessing whether the computed hill frame moves according to the motion of the spacecraft.

    Args:
        initial_position[3] (m): initial position of the spacecraft w.r.t. the body/origin
        initial_velocity[3] (m): initial velocity of the spacecraft w.r.t. the body/origin
        filter_dt (s): time between two consecutive reads of the input message
        orbit_normal_sign (-): sign of the reference frame "out of plane" vector (orbit normal or anti orbit normal)

    **Description of Variables Being Tested**

    The reference attitude :math:`\sigma_\mathcal{R/N}`, reference angular rates :math:`\omega_\mathcal{R/N}` and
    angular accelerations :math:`\dot{\omega}_\mathcal{R/N}` are tested. These are compared to the analytical results
    expected from the rectilinear motion described in the documentation of :ref:`flybyPoint`.
    The reference attitude is mapped to the corresponding reference frame, and each axis of the reference frame is
    tested for correctness. The angular rate and acceleration vectors are tested against the analytical result,
    expressed in R-frame coordinates.
    """
    # each test method requires a single assert method to be called
    flybyPointTestFunction(show_plots, initial_position, initial_velocity, filter_dt, orbit_normal_sign,
                           max_rate, max_acceleration)


def flybyPointTestFunction(show_plots, initial_position, initial_velocity, filter_dt, orbit_normal_sign,
                           max_rate, max_acceleration):
    # setup simulation environment
    sim_dt = 10
    unit_test_sim = SimulationBaseClass.SimBaseClass()
    process_rate = macros.sec2nano(sim_dt)
    test_process = unit_test_sim.CreateNewProcess("unit_process")
    test_process.addTask(unit_test_sim.CreateNewTask("unit_task", process_rate))

    # setup flybyPoint guidance module
    flyby_guidance = flybyPoint.FlybyPoint()
    flyby_guidance.ModelTag = "flybyPoint"
    flyby_guidance.setTimeBetweenFilterData(filter_dt)
    flyby_guidance.setToleranceForCollinearity(1E-5)
    flyby_guidance.setSignOfOrbitNormalFrameVector(orbit_normal_sign)
    flyby_guidance.setMaximumRateThreshold(max_rate)
    flyby_guidance.setMaximumAccelerationThreshold(max_acceleration)
    unit_test_sim.AddModelToTask("unit_task", flyby_guidance)

    input_data = messaging.NavTransMsgPayload()
    input_data.v_BN_N = np.array(initial_velocity)
    filter_msg = messaging.NavTransMsg()
    flyby_guidance.filterInMsg.subscribeTo(filter_msg)

    # Setup data logging before the simulation is initialized
    attitude_reference_log = flyby_guidance.attRefOutMsg.recorder()
    unit_test_sim.AddModelToTask("unit_task", attitude_reference_log)

    unit_test_sim.InitializeSimulation()
    position_data = []
    velocity_data = []
    for i in range(round(9 * 600 / sim_dt)):
        position_data.append(np.array(initial_position) + np.array(initial_velocity) * (i * sim_dt))
        velocity_data.append(np.array(initial_velocity))
        input_data.timeTag = macros.sec2nano(i * sim_dt)
        input_data.r_BN_N = position_data[i]
        filter_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * sim_dt) - 1)
        unit_test_sim.ExecuteSimulation()

    #  retrieve the logged data
    reference_attitude = attitude_reference_log.sigma_RN
    reference_rate = attitude_reference_log.omega_RN_N
    reference_acceleration = attitude_reference_log.domega_RN_N
    time_data = attitude_reference_log.times() * macros.NANO2MIN

    ur_output = []
    ut_output = []
    uh_output = []
    for i in range(len(reference_attitude)):
        RN = rbk.MRP2C(reference_attitude[i])
        ur_output.append(np.matmul(RN.transpose(), [1, 0, 0]))
        ut_output.append(np.matmul(RN.transpose(), [0, 1, 0]))
        uh_output.append(np.matmul(RN.transpose(), [0, 0, 1]))

    ur = initial_position / np.linalg.norm(initial_position)
    uh = np.cross(initial_position, initial_velocity) / np.linalg.norm(np.cross(initial_position, initial_velocity))
    ut = np.cross(uh, ur)
    f0 = np.linalg.norm(initial_velocity) / np.linalg.norm(initial_position)
    gamma0 = np.arctan(np.dot(initial_velocity, ur) / np.dot(initial_velocity, ut))

    for i in range(len(reference_attitude)):
        ur = position_data[i] / np.linalg.norm(position_data[i])
        uh = np.cross(position_data[i], velocity_data[i]) / np.linalg.norm(np.cross(position_data[i], velocity_data[i]))
        ut = np.cross(uh, ur)
        dt = time_data[i] * 60
        den = ((f0 * dt) ** 2 + 2 * f0 * np.sin(gamma0) * dt + 1)
        omega = uh * f0 * np.cos(gamma0) / den
        omegaDot = uh * (-2 * f0 * f0 * np.cos(gamma0)) * (f0 * dt + np.sin(gamma0)) / den / den
        if orbit_normal_sign == -1:
            ut = np.cross(ur, uh)
            uh = np.cross(ur, ut)

        # test correctness of frame, angular rate and acceleration
        np.testing.assert_allclose(ur_output[i], ur, rtol=0, atol=1E-12, verbose=True)
        np.testing.assert_allclose(ut_output[i], ut, rtol=0, atol=1E-12, verbose=True)
        np.testing.assert_allclose(uh_output[i], uh, rtol=0, atol=1E-12, verbose=True)
        np.testing.assert_allclose(reference_rate[i], omega, rtol=0, atol=1E-12, verbose=True)
        np.testing.assert_allclose(reference_acceleration[i], omegaDot, rtol=0, atol=1E-12, verbose=True)

    plt.close("all")
    position_data = np.array(position_data)
    velocity_data = np.array(velocity_data)
    plot_position(time_data, position_data)
    plot_velocity(time_data, velocity_data)
    plot_ref_attitude(time_data, reference_attitude)
    plot_ref_rates(time_data, reference_rate)
    plot_ref_accelerations(time_data, reference_acceleration)

    if show_plots:
        plt.show()

    plt.close("all")


def plot_position(time_data, position_data):
    """Plot the attitude errors."""
    plt.figure(1)
    for idx in range(3):
        plt.plot(time_data, position_data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$r_{BN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Position [m]')


def plot_velocity(time_data, velocity_data):
    """Plot the attitude errors."""
    plt.figure(2)
    for idx in range(3):
        plt.plot(time_data, velocity_data[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$v_{BN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Inertial Velocity [m/s]')


def plot_ref_attitude(time_data, sigma_RN):
    """Plot the attitude errors."""
    plt.figure(3)
    for idx in range(3):
        plt.plot(time_data, sigma_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference attitude')


def plot_ref_rates(time_data, omega_RN):
    """Plot the attitude errors."""
    plt.figure(4)
    for idx in range(3):
        plt.plot(time_data, omega_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference rates')


def plot_ref_accelerations(time_data, omegaDot_RN):
    """Plot the attitude errors."""
    plt.figure(5)
    for idx in range(3):
        plt.plot(time_data, omegaDot_RN[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\dot{\omega}_{RN,' + str(idx + 1) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Reference accelerations')


if __name__ == "__main__":
    test_flybyPoint(True,  # show_plots
                    [-5e7, 7.5e6, 5e5],  # initial_position
                    [2e4, 0, 0],  # initial_velocity
                    1,  # filter_dt
                    1,  # sign Orbit Normal
                    0.01,  # max rate
                    0  # max acceleration
                    )
