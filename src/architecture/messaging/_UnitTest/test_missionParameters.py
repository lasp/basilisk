"""The mission parameters get to the build along two paths.

The two paths can give different values, and the build gives no error message.

The C++ compiler reads the ``#include`` directives and finds ``mission/parameters.h``. SWIG does
not read ``#include`` directives. SWIG reads only the files that a ``.i`` file ``%include``s.
Thus the constants that SWIG gives to Python come from a different preprocessor. Only convention
makes sure that the two preprocessors read the same header.

When the two paths give different values, no failure occurs. The compiler sets the layout of the
payload structs, and Python shows a different bound. The results are then incorrect, but only
after a struct with the incorrect layout moves across an FFI boundary. The known cause is a digit
separator in the header. SWIG discards every constant after the separator, and Python then has a
missing attribute.

Each payload field below is a Python list. The length of the list comes from the compiled struct,
but the constant comes from the SWIG preprocessor. Thus a comparison of the two values makes sure
that the two paths agree.
"""

import pytest

from xmera.architecture import messaging

# One (payload type, field, constant) triple for each constant that gives an array bound.
SIZED_FIELDS = [
    ("RWSpeedMsgPayload", "wheelSpeeds", "RW_EFF_CNT"),
    ("RWArrayConfigMsgPayload", "JsList", "RW_EFF_CNT"),
    ("RwMotorTorqueMsgPayload", "motorTorque", "RW_EFF_CNT"),
    ("CSSArraySensorMsgPayload", "CosValue", "MAX_NUM_CSS_SENSORS"),
    ("THRArrayOnTimeCmdMsgPayload", "OnTimeRequest", "MAX_EFF_CNT"),
    ("SunlineFilterMsgPayload", "postFitRes", "MAX_NUM_CSS_SENSORS"),
    ("RegionsIdentifiedMsgPayload", "timeTag", "MAX_NUMBER_REGIONS"),
]

CONSTANTS = [
    "MAX_KEY_POINTS",
    "MAX_NUM_CSS_SENSORS",
    "MAX_EFF_CNT",
    "RW_EFF_CNT",
    "MAX_SICP_POINTS",
    "SICP_POINT_DIM",
    "MAX_SICP_ITERATIONS",
    "MAX_NUMBER_REGIONS",
]


@pytest.mark.parametrize("name", CONSTANTS)
def test_constant_reaches_python(name):
    """SWIG gives every mission constant to Python.

    If a constant is missing, the SWIG preprocessor discarded it. Usually the cause is text
    before that constant in the header, and not a constant that no code uses.
    """
    assert hasattr(messaging, name), (
        f"{name} did not get to the Python layer. SWIG stopped at some line in "
        f"mission/parameters.h and discarded the constants after that line."
    )
    assert isinstance(getattr(messaging, name), int)
    assert getattr(messaging, name) > 0


@pytest.mark.parametrize("payload,field,constant", SIZED_FIELDS)
def test_payload_extent_matches_constant(payload, field, constant):
    """The compiled struct and the SWIG constant agree on the array bound."""
    expected = getattr(messaging, constant)
    actual = len(getattr(getattr(messaging, payload)(), field))
    assert actual == expected, (
        f"The build compiled {payload}.{field} with {actual} elements, but Python shows "
        f"{constant} == {expected}. C++ and SWIG resolved different mission parameters. Make "
        f"sure that the build selected the correct mission/parameters.h. Then examine the "
        f"%include lines in the .i template."
    )
