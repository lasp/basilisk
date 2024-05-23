Executive Summary
-----------------
This document describes the lookup table's module in Basilisk software. The purpose of this module is to compute the 
corresponding motor 1 and motor 2 angles from the gimbal lookup table by using bilinear interpoation of the desired tip and tilt angles.

Message Connection Descriptions
-------------------------------
The following table lists all the module input and output messages.
The module msg variable names are set by the user from C.
The msg type contains a link to the message structure definition, while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - desiredGimbalTipAngleInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Input message of desired tip angle (theta)
    * - desiredGimbalTiltAngleInMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Input message of desired tilt angle (theta)
    * - motor1AngleOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message of desired motor 1 angle (theta)
    * - motor2AngleOutMsg
      - :ref:`HingedRigidBodyMsgPayload`
      - Output message of desired motor 2 angle (theta)



Detailed Module Description
---------------------------
The sepGimbalLookUppTable module reads 2 csv files, where each conatin tip and tilt angles with their corresponding motor angles. 

The module will recieve input messages of desired tip and tilt angles from the sepGimbal module, then it searches for the closest 2 
values of tip angle and store them in an array. Similrly, it searches for the closest 2 tilt angles and stores them in another array.

Subsequently, it extracts the corresponding motor angle for each set of tip and tilt angles, resulting in 4 motor angles corresponding to 4 sets of 
tip and tilt angles, such that the module is able to use the bilinear interpolation to find the coresponding motor 1 and 2 angles of the desired tip and tilt angles. 

A Delone triangulation method is also used for the edge cases of the diamond shaped motor angle values where a linear grid is not defined.
This mehod finds the nearest 3 values of tip and tilt instead of 4 and then determine the motor angle value. 

Finally, the motor angle 1 and 2 are outputted as messages to be sent to other modules.