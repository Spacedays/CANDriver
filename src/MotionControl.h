// #include "Arduino.h"
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "ControlInterface.h"
#include "math.h"
#include "Servo.h"

// #TODO: acceleration planning/steps

/// @brief Angle offsets for all servos [degrees]

const float SERVO_DT_MAX = 20;	// Max servo velocity [deg/s]

struct ServoQuad
{
	Servo FL;
	Servo FR;
	Servo BL;
	Servo BR;
	ServoQuad() = default;
};

struct ServoAngles
{
	uint16_t FL;
	uint16_t FR;
	uint16_t BL;
	uint16_t BR;
	ServoAngles() = default;
};

class RoverState
{
	public:
		RoverState() = default;
		
		void SetVector(MotionVector *mvec);
		
		void SetSteering(MotionVector *mvec);

		void SetSpeeds(MotionVector *mvec);

		void SetSpeedScale(float speedScale);

		void SetSteerOffsets(int8_t FL, int8_t FR, int8_t BL, int8_t BR);

		void Update();

		int InitServos(uint8_t pinFL, uint8_t pinFR, uint8_t pinBL, uint8_t pinBR);

		void CommandServos();

		ServoQuad driveServos;
		ServoAngles driveOffsets;
		MotionVector driveVec;
		MotionVector targetVec;
		int16_t SCd, SCh;
		unsigned long last_t;
		bool servos_enabled = false;
		// ServoAngles targetAngles = {};
		// int16_t targetThrottles = {};
		// LEDState ledState;

};

#endif