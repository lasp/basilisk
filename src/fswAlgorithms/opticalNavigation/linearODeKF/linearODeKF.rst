Executive Summary
-----------------

This module filters position measurements that have been processed from planet images in order to
estimate spacecraft relative position to an observed body in the inertial frame.
The filter used is a linear or extended Kalman filter, and the images are first processed by image processing
and a measurement model in order to produce this filter's measurements.

The module inherits from the EKF interface and implements only the few virtual methods.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.  The module msg connection is set by the
user from python.  The msg type contains a link to the message structure definition, while the description
provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - navTransOutMsg
      - :ref:`NavTransMsgPayload`
      - navigation translation output message
    * - opNavFilterMsg
      - :ref:`FilterMsgPayload`
      - output filter data message containing states and covariances
    * - opNavResidualMsg
      - :ref:`FilterResidualMsgPayload`
      - output measurement data message containing residuals
    * - opNavHeadingMsg
      - :ref:`OpNavUnitVecMsgPayload`
      - opnav input message containing the unit vector towards the target

Module models
-------------------------------
The measurement model for the filter is functionally contained in the center of brightness
converter module. The message read in therefore contains the predicted measurement:

.. math::

    H[X] = \frac{X[0:3]}{\|X[0:3]\|} = \hat{r}

The dynamics modules used are point mass, single body gravity, propagated with and RK4
integrator

.. math::

    F[X] = \dot{X} = v

where :math:`r` is the position of the spacecraft center of mass relative to the central body. The velocity is either
set and held constant or estimated by the filter given no acceleration.


Module assumptions and limitations
-------------------------------

.. list-table:: Interface methods implemented
    :widths: 25 75 50
    :header-rows: 1

    * - Method Name
      - Method Function
      - Class specifics
    * - customReset
      - perform addition reset duties in child class
      - assert message link and convert gravitational parameter units
    * - readFilterMeasurements
      - read the specific measurements in a child class
      - read opnavHeading message
    * - writeOutputMessages
      - write the specific measurements in a child class
      - write Nav message and reconvert units
    * - measurementModel
      - add a measurement model for the inputs in child class
      - read opnav heading message and normalize
    * - propagate
      - add a dynamics model for the inputs in child class
      - use two body gravity and and rk4 to propagate


User Guide
----------
This section is to outline the steps needed to setup a flybyODSRuKF converter in Python.

#. Import the module::

    from Basilisk.fswAlgorithms import linearODeKF

#. Create an instantiation of converter class::

    flybyOD = linearODeKF.LinearODeKF()

#. Setup EKF general parameters::

    flybyOD.setAlpha(0.02)
    flybyOD.setBeta(2.0)

#. Setup EKF measurement parameters::

    flybyOD.setMeasNoiseScaling(1)

#. Setup initial state and covariances to estimate position and velocity::

    flybyOD.setInitialPosition([1000.*1e3, 1000.*1e3, 1000.*1e3])
    flybyOD.setInitialVelocity([0., 1.*1e3, 0.])
    flybyOD.setInitialCovariance([ [10., 0., 0., 0., 0., 0.],
                             [0., 10., 0., 0., 0., 0.],
                             [0., 0., 10., 0., 0., 0.],
                             [0., 0., 0., 0.01, 0., 0.],
                             [0., 0., 0., 0., 0.01, 0.],
                             [0., 0., 0., 0., 0., 0.01]])

#. Or, setup initial state and covariances to estimate just position ::

    flybyOD.setInitialPosition([1000.*1e3, 1000.*1e3, 1000.*1e3])
    flybyOD.setConstantVelocity([0., 1.*1e3, 0.])
    flybyOD.setInitialCovariance([ [10., 0., 0.],
                             [0., 10., 0.],
                             [0., 0., 10.],
                             ])

#. Setup process noise::

    sigmaPos = 0.01
    sigmaVel = 0.0001
    flybyOD.setProcessNoise([[sigmaPos, 0., 0., 0., 0., 0.],
                      [0., sigmaPos, 0., 0., 0., 0.],
                      [0., 0., sigmaPos, 0., 0., 0.],
                      [0., 0., 0., sigmaVel, 0., 0.],
                      [0., 0., 0., 0., sigmaVel, 0.],
                      [0., 0., 0., 0., 0., sigmaVel]])

#. Subscribe to the messages, primarily the measurement message::

    flybyOD.opNavHeadingMsg.subscribeTo(cobConverter.opnavUnitVecOutMsg)
