# ISC License
#
# Copyright (c) 2024, Laboratory for Atmospheric and Space Physics,
# University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.


import numpy as np
import pytest
import linearODeKF_test_utilities as filter_plots

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import linearODeKF, ekfInterface
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.architecture import rigidBodyKinematics as rbk


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def rk4(f, t, x0, mu):
    x = np.zeros([len(t), len(x0) + 1])
    h = (t[len(t) - 1] - t[0]) / len(t)
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        x[i, 0] = t[i]
        k1 = h * f(t[i], x[i, 1:], mu)
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1, mu)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2, mu)
        k4 = h * f(t[i] + h, x[i, 1:] + k3, mu)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2. * k2 + 2. * k3 + k4) / 6.
        x[i + 1, 0] = t[i + 1]
    return x


def two_body_gravity(t, x, velocity):
    dxdt = np.zeros(np.shape(x))
    if velocity is None:
        velocity = x[3:]
    else:
        velocity = [8528.685319524433, -4924.038765061039, -1736.4817766693034]
    dxdt[0:3] = velocity
    return dxdt


def setup_filter_data(type = "classical", velocity = False):
    # Construct algorithm and associated C++ container
    ckf = ekfInterface.FilterType_Classical
    ekf = ekfInterface.FilterType_Extended
    if type == "classical":
        filter_object = linearODeKF.LinearODeKF(ckf)
    if type == "extended":
        filter_object = linearODeKF.LinearODeKF(ekf)
        filter_object.setMinimumCovarianceNormForEkf(1E5)  # filter in km and km/s

    closest = 500*1e3
    dT = 25 * 60
    v_inf = 10*1e3
    # initial state in flyby frame F
    r_F = [-v_inf * dT, -closest, 0]
    v_F = [v_inf, 0, 0]

    dcm_FN = np.array(rbk.eulerAngles321ToDcm([-30*np.pi/180, 10*np.pi/180, 0.0]))
    dcm_NF = dcm_FN.transpose()

    r_N = np.dot(dcm_NF, np.array(r_F)).tolist()
    v_N = np.dot(dcm_NF, np.array(v_F)).tolist()
    if not velocity:
        filter_object.setUnitConversionFromSItoState(1E-3)  # filter in km and km/s
        filter_object.setInitialPosition(r_N)
        filter_object.setInitialVelocity(v_N)
        filter_object.setInitialCovariance([[1000. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0],
                                            [0.0, 100. * 1E6, 0.0, 0.0, 0.0, 0.0],
                                            [0.0, 0.0, 100. * 1E6, 0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0, 0.1 * 1E6, 0.0, 0.0],
                                            [0.0, 0.0, 0.0, 0.0, 0.01 * 1E6, 0.0],
                                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.01 * 1E6]])

        sigmaVel = (1E-4) ** 2
        filter_object.setProcessNoise([[sigmaVel, 0.0, 0.0],
                                       [0.0, sigmaVel*10, 0.0],
                                       [0.0, 0.0, sigmaVel*10]])
    else:
        filter_object.setUnitConversionFromSItoState(1E-3)  # filter in km and km/s
        filter_object.setInitialPosition(r_N)
        filter_object.setConstantVelocity(v_N)
        filter_object.setInitialCovariance([[1000. * 1E6, 0.0, 0.0],
                                            [0.0, 100. * 1E6, 0.0],
                                            [0.0, 0.0, 100. * 1E6]])

        sigmaPosition = (1E-2) ** 2
        filter_object.setProcessNoise([[sigmaPosition, 0.0, 0.0],
                                       [0.0, sigmaPosition*10, 0.0],
                                       [0.0, 0.0, sigmaPosition*10]])
    return filter_object

@pytest.mark.parametrize("dt", [1, 10])
@pytest.mark.parametrize("filter_type", ["classical", "extended"])
@pytest.mark.parametrize("constant_velocity", [False, True])
def test_propagation_linear_kf(show_plots, dt, filter_type, constant_velocity):
    """Module Unit Test"""
    state_propagation_flyby(show_plots, dt, filter_type, constant_velocity)

@pytest.mark.parametrize("initial_errors", [0, 1])
@pytest.mark.parametrize("filter_type", ["classical", "extended"])
@pytest.mark.parametrize("constant_velocity", [False, True])
def test_measurements_linear_kf(show_plots, filter_type, constant_velocity, initial_errors):
    """Module Unit Test"""
    state_update_flyby(show_plots, filter_type, constant_velocity, initial_errors)


def state_propagation_flyby(show_plots, dt, filter_type, constant_velocity):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    flyby_module = setup_filter_data(filter_type, constant_velocity)

    unit_test_sim.AddModelToTask(unit_task_name, flyby_module)

    filter_data_log = flyby_module.opNavFilterMsg.recorder()
    nav_data_log = flyby_module.navTransOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, nav_data_log)

    opnav_input_msg = messaging.OpNavUnitVecMsg()
    flyby_module.opNavHeadingMsg.subscribeTo(opnav_input_msg)

    sim_time = 30
    time = np.linspace(0, int(sim_time * 60), int(sim_time * 60 // dt) + 1)
    expected = np.zeros([len(time), 7])
    expected[0, 1:4] = np.array(flyby_module.getInitialPosition()).reshape(3)
    if constant_velocity:
        expected[0, 4:7] = [8528.685319524433, -4924.038765061039, -1736.4817766693034]
    else:
        expected[0, 4:7] = np.array(flyby_module.getInitialVelocity()).reshape(3)
    expected = rk4(two_body_gravity, time, expected[0, 1:], constant_velocity)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.min2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    num_states = filter_data_log.numberOfStates[0]
    r_data_log = add_time_column(nav_data_log.times(), nav_data_log.r_BN_N)
    v_data_log = add_time_column(nav_data_log.times(), nav_data_log.v_BN_N)
    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    diff = np.copy(expected)
    diff[:, 1:4] -= r_data_log[:, 1:]
    diff[:, 4:7] -= v_data_log[:, 1:]
    filter_plots.state_covar(state_data_log, covariance_data_log, 'Prop', show_plots)
    filter_plots.states(diff, 'Prop', show_plots)

    np.testing.assert_array_less(np.linalg.norm(covariance_data_log[0, 1:]),
                                 np.linalg.norm(covariance_data_log[-1, 1:]),
                                 err_msg='covariance must increase without measurements',
                                 verbose=True)
    np.testing.assert_allclose(diff[:, 1:],
                               np.zeros(np.shape(diff[:, 1:])),
                               atol=1E-6,
                               err_msg='state propagation error',
                               verbose=True)
    np.testing.assert_allclose(state_data_log[:, 1:4],
                           r_data_log[:, 1:],
                           rtol=1E-2,
                           atol=1E-8,
                           err_msg='message writing error',
                           verbose=True)
    v_data_log = add_time_column(nav_data_log.times(), nav_data_log.v_BN_N)
    if constant_velocity:
        np.testing.assert_allclose(np.reshape(expected[0, 4:7].tolist() * len(v_data_log[:, 0]), [len(v_data_log[:, 0]), 3]),
                           v_data_log[:, 1:],
                           rtol=1E-2,
                           atol=1E-8,
                           err_msg='message writing error',
                           verbose=True)

    if not constant_velocity:
        np.testing.assert_allclose(state_data_log[:, 4:num_states+1],
                           v_data_log[:, 1:],
                           rtol=1E-2,
                           atol=1E-8,
                           err_msg='message writing error',
                           verbose=True)

def state_update_flyby(show_plots, filter_type, constant_velocity, errors):
    unit_task_name = "unit_task"
    unit_process_name = "test_process"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dt = 1.0
    t1 = 250
    multT1 = 8

    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Add test module to runtime call list
    flyby_module = setup_filter_data(filter_type, constant_velocity)
    unit_test_sim.AddModelToTask(unit_task_name, flyby_module)

    filter_data_log = flyby_module.opNavFilterMsg.recorder()
    residual_data_log = flyby_module.opNavResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, residual_data_log)
    time = np.linspace(0, int(multT1 * t1), int(multT1 * t1 // dt) + 1)
    if not constant_velocity:
        initial_errors = np.array([1, 5, 10, 0.01, 0.02, -0.01])*1e3
        expected = np.zeros([len(time), 7])
        expected[0, 1:4] = np.array(flyby_module.getInitialPosition()).reshape(3) - initial_errors[:3]*errors
        expected[0, 4:7] = np.array(flyby_module.getInitialVelocity()).reshape(3) - initial_errors[3:]*errors
        kick = np.array([0.2, 1, 5, 0.01, 0.02, 0.03]) * 1E2
    else:
        initial_errors = np.array([1, 5, 10])*1e3
        expected = np.zeros([len(time), 4])
        expected[0, 1:4] = np.array(flyby_module.getInitialPosition()).reshape(3) - initial_errors*errors
        kick = np.array([0.2, 1, 5]) * 1E2

    expected[0:t1, :] = rk4(two_body_gravity, time[0:t1], expected[0, 1:], constant_velocity)
    expected[t1-1:multT1 * t1 + 1, :] = rk4(two_body_gravity, time[t1-1:len(time)+1], expected[t1 - 1, 1:] + kick, constant_velocity)

    input_data = messaging.OpNavUnitVecMsgPayload()
    opnav_input_msg = messaging.OpNavUnitVecMsg()
    flyby_module.opNavHeadingMsg.subscribeTo(opnav_input_msg)
    input_data.rhat_BN_B = expected[0, 1:4] / np.linalg.norm(expected[0, 1:4])

    noise = np.zeros([len(time), 3])
    unit_test_sim.InitializeSimulation()

    noise_std = 1000
    input_data.covar_N = [noise_std**2/np.linalg.norm(expected[0, 1:4])**2, 0., 0.,
                          0., noise_std**2/np.linalg.norm(expected[0, 1:4])**2, 0.,
                          0., 0., noise_std**2/np.linalg.norm(expected[0, 1:4])**2]
    noise[0, :] = np.diagonal(np.array(input_data.covar_N).reshape([3,3]))
    for i in range(t1):
        if i > 0 and i % 10 == 0:
            input_data.timeTag = (i) * dt
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, noise_std, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [noise_std**2/np.linalg.norm(expected[i, 1:4])**2, 0., 0.,
                                  0., noise_std**2/np.linalg.norm(expected[i, 1:4])**2, 0.,
                                  0., 0., noise_std**2/np.linalg.norm(expected[i, 1:4])**2]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        noise[i, :] = np.diagonal(np.array(input_data.covar_N).reshape([3,3]))
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()


    num_states = filter_data_log.numberOfStates[0]
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])
    covariance_diagonal = np.zeros([len(filter_data_log.times()), 3])
    if constant_velocity:
        covariance_diagonal[:, 0] = covariance_data_log[:, 1]
        covariance_diagonal[:, 1] = covariance_data_log[:, num_states+1 + 1]
        covariance_diagonal[:, 2] = covariance_data_log[:, 2*num_states+2 + 1]
    else:
        covariance_diagonal[:, 0] = covariance_data_log[:, 3*num_states + 3 + 1]
        covariance_diagonal[:, 1] = covariance_data_log[:, 4*num_states + 4 + 1]
        covariance_diagonal[:, 2] = covariance_data_log[:, 5*num_states + 5 + 1]
    np.testing.assert_array_less(covariance_diagonal[t1, :],
                                 covariance_diagonal[0, :],
                                  err_msg='covariance error',
                                  verbose=True)


    for i in range(t1, multT1 * t1):
        if i % 10 == 0:
            input_data.timeTag = (i) * dt
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, noise_std, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [noise_std**2/np.linalg.norm(expected[i, 1:4])**2, 0., 0.,
                                  0., noise_std**2/np.linalg.norm(expected[i, 1:4])**2, 0.,
                                  0., 0., noise_std**2/np.linalg.norm(expected[i, 1:4])**2]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        noise[i, :] = np.diagonal(np.array(input_data.covar_N).reshape([3,3]))
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    stateError_data_log = add_time_column(filter_data_log.times(), filter_data_log.stateError[:, :num_states])
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])
    number_obs = residual_data_log.numberOfObservations
    size_obs = residual_data_log.sizeOfObservations
    post_fit_log_sparse = add_time_column(residual_data_log.times(), residual_data_log.postFits)
    post_fit_log = np.zeros([len(residual_data_log.times()), np.max(size_obs)+1])
    post_fit_log[:, 0] = post_fit_log_sparse[:, 0]
    pre_fit_log_sparse = add_time_column(residual_data_log.times(), residual_data_log.preFits)
    pre_fit_log = np.zeros([len(residual_data_log.times()), np.max(size_obs)+1])
    pre_fit_log[:, 0] = pre_fit_log_sparse[:, 0]

    for i in range(len(number_obs)):
        if number_obs[i] > 0:
            post_fit_log[i, 1:size_obs[i]+1] = post_fit_log_sparse[i, 1:size_obs[i]+1]
            pre_fit_log[i, 1:size_obs[i]+1] = pre_fit_log_sparse[i, 1:size_obs[i]+1]

    diff = np.copy(state_data_log)[:, 0:num_states+1]
    diff[:, 1:] -= expected[:, 1:num_states+1]
    filter_plots.state_covar(state_data_log, covariance_data_log, 'Update-state', show_plots)
    filter_plots.state_covar(stateError_data_log, covariance_data_log, 'Update-xbar', show_plots)
    filter_plots.state_covar(diff, covariance_data_log, 'Update-error', show_plots)
    filter_plots.states(diff, 'Update', show_plots)
    filter_plots.post_fit_residuals(pre_fit_log, noise, 'Update PreFit', show_plots)
    filter_plots.post_fit_residuals(post_fit_log, noise, 'Update PostFit', show_plots)

    np.testing.assert_array_less(np.diag(covariance_data_log[t1 * multT1, 1:].reshape([num_states, num_states]))[1:],
                                 np.diag(covariance_data_log[0, 1:].reshape([num_states, num_states]))[1:],
                                 err_msg='covariance error',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[-1, 1:],
                               expected[-1, 1:],
                               atol=5*noise_std,
                               err_msg='state propagation error',
                               verbose=True)


if __name__ == "__main__":
    state_update_flyby(True, "classical", True, 1)
