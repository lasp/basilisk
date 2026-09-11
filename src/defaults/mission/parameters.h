#ifndef MISSION_PARAMETERS_H
#define MISSION_PARAMETERS_H

// These are the default mission parameters. The build uses them when no module root declares its
// own parameters with xmera_provide_mission_parameters(). A mission supplies its own header, and
// that header must give a value for every constant. The build does not add these values to that
// header. Refer to docs/source/learn/making-modules/messaging/mission-parameters.rst.

// SWIG also reads this header, to give the constants to the Python layer. Its preprocessor reads
// the quote in a digit separator, for example 5'000, as the start of a character literal. SWIG
// then discards every constant after that quote, and it gives no error message. The
// clang-format off guard makes sure that IntegerLiteralSeparator does not add the separators
// again.

// clang-format off
#define MAX_KEY_POINTS 5000
#define MAX_NUM_CSS_SENSORS 32
#define MAX_EFF_CNT 36
#define RW_EFF_CNT 36

#define MAX_SICP_POINTS 5000
#define SICP_POINT_DIM 3
#define MAX_SICP_ITERATIONS 250

#define MAX_NUMBER_REGIONS 3
// clang-format on

#endif  // MISSION_PARAMETERS_H
