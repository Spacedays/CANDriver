#define CONTROLTEST

#ifdef CONTROLTEST
#include "ControlInterface.cpp"
#elif defined(DUAL)
#include "../Brain/CANBrain_dualcore.cpp"
#elif defined(BRAIN)
#include "../Brain/CANBrain.cpp"
#else
#include "../Driver/CANDriver.cpp"
#endif