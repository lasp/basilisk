Executive Summary
-----------------

This module reads in the Reaction Wheel (RW) speeds, determines the net RW momentum, and then determines the amount of angular momentum that must be dumped.

A separate thruster firing logic module called thrMomentumDumping will later on compute the thruster on cycling. The module
:download:`PDF Description </../../src/fswAlgorithms/attControl/thrMomentumManagement/_Documentation/Basilisk-thrMomentumManagement-20160817.pdf>`
contains further information on this module's function, how to run it, as well as testing.

Module Input and Output
=======================

The module input and output messages are illustrated in Figure~\ref{fig:moduleImg}.  The module has a single output message of type {\tt CmdTorqueBodyIntMsg} which contains the desired momentum change $\leftexp{B}{\Delta}\bm H$.

There are 2 required input messages.  The message of type {\tt RWArrayConfigMsg} is read in during the {\tt Reset()} method and provides the RW configuration states.  The message of type {\tt RWSpeedMsg} provides the current RW speeds and is read in during the {\tt Update()} method.

Table `1 <#tab:inputRWSpeedTable>`__ shows the input message with the reaction wheel speed information.

.. container::
   :name: tab:inputRWSpeedTable

   .. table:: Input RW Speed Message

      +--------------------------+-----------+--------+------------------------------+
      | Name                     | Type      | Length | Description                  |
      +==========================+===========+========+==============================+
      | :math:`\Omega`           | double    | 1      | Angular velocity of the      |
      |                          |           |        | reaction wheels              |
      +--------------------------+-----------+--------+------------------------------+
      | :math:`\theta`           | double    | 1      | Angle of the reaction wheels |
      +--------------------------+-----------+--------+------------------------------+

Table `2 <#tab:inputRWArrayTable>`__ shows the input message about the reaction wheel array information.

.. container::
   :name: tab:inputRWArrayTable

   .. table:: Input RW Array Message

      +--------------------------+-----------+--------+------------------------------+
      | Name                     | Type      | Length | Description                  |
      +==========================+===========+========+==============================+
      | :math:`\boldsymbol{G}_S` | double [] | 3      | Reaction wheel spin axis     |
      +--------------------------+-----------+--------+------------------------------+
      | :math:`J_S`              | double    | 1      | Reaction wheel inertia about |
      |                          |           |        | its spin axismmmmmmm         |
      +--------------------------+-----------+--------+------------------------------+
      | :math:`n_{\text{RW}}`    | int       | 1      | Number of reaction wheels    |
      +--------------------------+-----------+--------+------------------------------+
      | :math:`u_{\text{max}}`   | double    | 1      | Maximum motor torque         |
      +--------------------------+-----------+--------+------------------------------+

Table `3 <#tab:outputTorqueTable>`__ shows the output message of the module.

.. container::
   :name: tab:outputTorqueTable

   .. table:: Output Torque Message

      +--------------------------+-----------+--------+------------------------------+
      | Name                     | Type      | Length | Description                  |
      +==========================+===========+========+==============================+
      | :math:`\boldsymbol{L}_r` | double [] | 3      | Requested torque applied to  |
      |                          |           |        | the hub as an external torque|
      +--------------------------+-----------+--------+------------------------------+

Module Description
==================

To manage the Reaction Wheel (RW) angular momentum build-up over time, a thruster-based momentum dumping strategy is used. The output of {\tt thrMomentumManagement} module is a :math:`\Delta \boldsymbol H` vector.  This output is then mapped into a thruster impulse request using the {\tt thrForceMapping} module.  Note that this latter module is designed to map a control torque vector into thruster forces.  If the input torque and output force sets are multiplied by time,  the same module also functions to map a desired angular momentum changes vector :math:`\Delta H` into a set of thruster impulse requests.  The final module {\tt thrMomentumDumping} in the series takes the thruster impulse requests and determines a thruster firing sequence to achieve this desired momentum change.  The spacecraft attitude is held constant by simultaneously having a RW control module holding an inertial attitude.  The process of holding the desired attitude leads to the RWs despinning to the desired level due the external thruster disturbances.

Assume the spacecraft contains :math:`N_{\text{RW}}` RWs. The net RW angular momentum is given by

.. math::

  \begin{equation}
    \boldsymbol h_{s} = \sum_{i=1}^{N_{\text{RW}}} \hat{\boldsymbol g}_{s_{i}} J_{s_{i}} \Omega_{i}
  \end{equation}
where :math:`\hat{\boldsymbol g}_{s_{i}}` is the RW spin axis, :math:`J_{s_{i}}` is the spin axis RW inertia and :math:`\Omega_{i}` is the RW speed rate about this axis.
Because the inertial attitude of the spacecraft is assumed to be held nominally steady the body-relative RW cluster angular momentum rate can be approximated as

.. math::

  \begin{equation}
    \dot{\boldsymbol h}_{s} = \frac{{}^{\mathcal{B}}{\text{d}}\boldsymbol h_{s}}{\text{d} t} + \boldsymbol\omega_{\mathcal{B}/N} \times \boldsymbol h_{s} \approx \frac{{}^{\mathcal{B}}{\text{d}}\boldsymbol h_{s}}{\text{d} t}
  \end{equation}

Let :math:`h_{s,\text{min}}` be lower bound that the RW momentum dumping strategy should achieve.  The desired net change in inertial angular momentum is thus determined through

.. math::

  \begin{equation}
    {}^{\mathcal{B}}{\Delta}\boldsymbol H = -{}^{\mathcal{B}}{\boldsymbol h}_{s} \frac{
      |\boldsymbol h_{s}| - h_{s,\text{min}}
    }{|\boldsymbol h_{s}|}
  \end{equation}

This strategy requires a thruster firing solution which creates this desired :math:`{}^{\mathcal{B}}{\Delta}\boldsymbol H` over the duration of the momentum dumping.  The goal of the RW momentum management module is to simply compute if a :math:`{}^{\mathcal{B}}{\Delta}\boldsymbol H` is required, or set it equal to zero if the RW momentum is too small.  Not that this module will only compute :math:`{}^{\mathcal{B}}{\Delta}\boldsymbol H` once.  Either it is zero or non-zero. To reuse this momentum management module, the reset() function must be called.

The Reset Method
================

The ``Reset()`` method reads in the RW configuration message and then resets the flag to do the angular momentum checking.  The goal here is to do this momentum checking only once after the reset function is called, rather than doing this checking autonomously.

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
    * - deltaHOutMsg
      - :ref:`CmdTorqueBodyMsgPayload`
      - output message with the requested inertial angular momentum change
    * - rwSpeedsInMsg
      - :ref:`RWSpeedMsgPayload`
      - reaction wheel speed input message
    * - rwConfigDataInMsg
      - :ref:`RWArrayConfigMsgPayload`
      - name of the RWA configuration message
