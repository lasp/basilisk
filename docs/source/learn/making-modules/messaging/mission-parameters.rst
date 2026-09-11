.. _messaging-mission-parameters:

Configuring Mission Parameters
==============================

Several message payloads and modules get their array bounds from a small set of
project-wide constants. These constants include sensor counts and effector
counts. They are in one header:

``mission/parameters.h``

One header only is in use for a build. That header gives every constant. The
build does not add values to it, and it does not give a default value for a
missing constant.

The Mission Parameters
----------------------

============================ ======= ========================================
Constant                     Default Meaning
============================ ======= ========================================
``MAX_KEY_POINTS``           5000    Optical flow key points
``MAX_NUM_CSS_SENSORS``      32      Coarse sun sensors in a constellation
``MAX_EFF_CNT``              36      Generic effectors (thrusters and others)
``RW_EFF_CNT``               36      Reaction wheels
``MAX_SICP_POINTS``          5000    Point-cloud points
``SICP_POINT_DIM``           3       Point-cloud point dimension
``MAX_SICP_ITERATIONS``      250     Point-cloud registration iterations
``MAX_NUMBER_REGIONS``       3       Regions of interest
============================ ======= ========================================

Each constant gives a bound for a message payload array. Thus all the constants
are part of the messaging ABI, and SWIG gives all of them to the Python layer.
The default values are large. Thus a build with no mission of its own operates
correctly.

Where the Values Come From
--------------------------

The array bounds belong to the mission, and a mission comes to the build as a
module root. A module root declares its own values with this call:

.. code-block:: cmake

    xmera_provide_mission_parameters("${CMAKE_CURRENT_SOURCE_DIR}")

Put this call in the ``CMakeLists.txt`` of the module root. The argument is the
directory that *contains* a ``mission`` folder. When you add that root to
``XMERA_MODULE_ROOTS``, you also select the array bounds that its code uses.
The two are one selection. Thus they cannot become different, and there is no
second control to set.

The configure step shows the header that it resolved:

.. code-block:: text

    -- Mission parameters: /path/to/myMission/mission/parameters.h

One source only is permitted. If two module roots declare parameters, the
configure step stops and gives the two directories. This check is necessary.
Before this check, a build compiled its Python message payloads against one set
of bounds, and its algorithm objects against a different set. The build gave no
error message. The build showed incorrect results only when a struct moved
across an FFI boundary.

If no module root declares a directory, the build uses
``src/defaults/mission/parameters.h``. That header gives the default values in
the table above.

Supplying Your Own Values
-------------------------

Make a ``mission/parameters.h`` file in your module root. Give a value for
**every** constant in the table. Some of the constants are not in your own
code, but a payload in a different part of the build uses them:

.. code-block:: c

    #ifndef MISSION_PARAMETERS_H
    #define MISSION_PARAMETERS_H

    // clang-format off
    #define MAX_NUM_CSS_SENSORS 12
    #define MAX_EFF_CNT 12
    #define RW_EFF_CNT 4

    #define MAX_KEY_POINTS 5000
    #define MAX_SICP_POINTS 5000
    #define SICP_POINT_DIM 3
    #define MAX_SICP_ITERATIONS 250
    #define MAX_NUMBER_REGIONS 3
    // clang-format on

    #endif

Then declare the directory from the ``CMakeLists.txt`` of the module root, as
in the example above. The configure step does not find a missing constant. The
first payload or algorithm that uses that constant gives a compilation error.

Writing the Header
------------------

Write the values as plain integers, in the ``clang-format off`` guard in the
example above. SWIG also reads this header, to give the constants to the Python
layer. Its preprocessor reads the quote in a digit separator, for example
``5'000``, as the start of a character literal. SWIG then discards every
constant after that quote. The result is a missing attribute on
``xmera.architecture.messaging``, and not a build error.

The guard is necessary because the ``.clang-format`` file of this project sets
``IntegerLiteralSeparator``, which adds those separators for you.

``test_missionParameters.py`` prevents that class of failure. The test compares
the compiled array extent of each payload against the constant that SWIG gives
to Python.
