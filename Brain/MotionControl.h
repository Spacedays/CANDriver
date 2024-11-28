// #include "Arduino.h"
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "ControlInterface.h"
#include "math.h"
#include "Servo.h"

// #TODO: acceleration planning/steps

/// @brief Angle offsets for all servos [degrees]

struct ServoQuad
{
	Servo FL;
	Servo FR;
	Servo BL;
	Servo BR;
};

class RoverState
{
	public:
		void SetVector(MotionVector *mvec);
		
		void SetSteering(MotionVector *mvec);

		void SetSpeeds(MotionVector *mvec);

		void SetSpeedScale(float speedScale);

		void SetSteerOffsets(int8_t FL, int8_t FR, int8_t BL, int8_t BR);

		

		ServoQuad driveServos = {};
		ServoQuad driveOffsets = {};

		// LEDState ledState;

};

#endif