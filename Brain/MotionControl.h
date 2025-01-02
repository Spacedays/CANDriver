// #include "Arduino.h"
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "ControlInterface.h"
#include "math.h"
#include "Servo.h"

// #TODO: acceleration planning/steps

/// @brief Angle offsets for all servos [degrees]

const u8_t SERVO_DT_MAX = 20;	// Max servo velocity [deg/s]

struct ServoQuad
{
	Servo FL;
	Servo FR;
	Servo BL;
	Servo BR;
};

struct ServoAngles
{
	uint16_t FL;
	uint16_t FR;
	uint16_t BL;
	uint16_t BR;
};

class RoverState
{
	public:
		void SetVector(MotionVector *mvec);
		
		void SetSteering(MotionVector *mvec);

		void SetSpeeds(MotionVector *mvec);

		void SetSpeedScale(float speedScale);

		void SetSteerOffsets(int8_t FL, int8_t FR, int8_t BL, int8_t BR);

		int Update();		

		int InitServos(pin_size_t pinFL, pin_size_t pinFR, pin_size_t pinBL, pin_size_t pinBR);

		void CommandServos();

		ServoQuad driveServos = {};
		ServoAngles driveOffsets = {};
		MotionVector driveVec = {};
		MotionVector targetVec = {};
		int16_t SCd, SCh;
		unsigned long last_t;
		// ServoAngles targetAngles = {};
		// int16_t targetThrottles = {};
		// LEDState ledState;

};

#endif