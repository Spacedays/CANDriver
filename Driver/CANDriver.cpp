/*
The code for a SimpleFOC Motor Driver, controlled via CAN Bus
*/

#ifndef CANDRIVER_CPP
#define CANDRIVER_CPP
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include "SimpleFOC.h"
#include "ControlInterface.h"
#include "rover_config.h"


#pragma region HardwareConfig
#define DEBUG	// Enable serial console
// #define COMMANDER	// enable SFOC Commander (disables serial console)

#ifdef PICO
#pragma message("PICO - 17-CANTX 16-CANRX")
const uint8_t pinTx = 17;
const uint8_t pinRx = 16;

#elif defined(STM32G4xx)
// CAN pins
const uint8_t pinTx = PA12;
const uint8_t pinRx = PA11;

const TwoWire Wire2(PB9,PA15);
// const uint32_t SDApin = PB9;
// const uint32_t SCLpin = PA15;

const float SUPPLY_VOLTAGE = 9; 

#else
uint8_t pinTx = -1;
uint8_t pinRx = -1;
#endif

#pragma endregion HardwareConfig

// #ifndef DEVICEID
// 	#ifdef PICO
// 	#define DEVICEID 1
// 	#else
// 	#define DEVICEID 0
// 	#endif
// #endif
// uint8_t MyDeviceID=DEVICEID;
uint8_t MyDeviceID=get_driver_id();//DEVICEID;

#include "SimpleCAN.h"
#include "CANDriverProfile.h"


// HardwareSerial Serial3(PB11, PB10);   // uart3
// HardwareSerial Serial1(PB7, PB6);  // uart1
bool ledstatus;
uint32_t led_time = 0;

#pragma region SimpleFOC_Setup

#define DebugSerial Serial
// HardwareSerial Serial1(PB7, PB6);  // uart1
// #define DebugSerial Serial1
// HardwareSerial Serial3(PB11, PB10);   // uart3
// #define DebugSerial Serial3

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float velocity_target = 0.0;
bool monitoring_var = false; // bool to toggle debug printing

#ifdef COMMANDER
Commander command = Commander(DebugSerial);

bool* var_ref = &monitoring_var;
void toggle_monitoring(char* str){
  *var_ref = !*var_ref;
  if (*var_ref) { DebugSerial.println("monitoring enabled"); } else { DebugSerial.println("monitoring disabled"); }
}
void doMotor(char* cmd) { command.motor(&motor, cmd); }
void mdisable(char* cmd) { motor.disable(); DebugSerial.println("motor disabled"); led_time = 600;}
void menable(char* cmd) { motor.enable(); DebugSerial.println("motor enabled"); led_time = 300;}
void setTarget(char* cmd) { command.scalar(&velocity_target,cmd);}
#endif

// The integer throttle value is scaled using this
float speed_scale = 20.0;


#pragma endregion SimpleFOC_Setup

#pragma region CANSetup

// Function to sign-extend an integer up to 32 bits
int32_t signExtend(int32_t value, int bitLength) {
	int shift = 32 - bitLength;
	return (value << shift) >> shift;
}

// Broker class which handles information received from CAN bus.
// These functions are called from within CANPingPong::Can1->Loop() whenever there are new CAN messages in the queue.
// CAN IDs you want to react to must be specified in the CAN profile class (PingPongNotificationsFromCAN in this example).
// CANPingPong::Can1->Loop()   ->  CANPingPong::HandleCanMessage() ->  RxFromCAN::ReceivedXxxx()
// The reason for having this broker class is that it allows us to completely separate the profile definition from any code
// which is related to e.g. motor control. this means, a single header file is sufficient to define the profile and the same header file
// can be used on both sides of the CAN bus without having to include all the stuff which may be required on one side only.
class RxFromCAN : public CANDriverNotifications
{
	public:
		// RxFromCAN() : CANDriverNotifications(){};
		// RxFromCAN() : CANDriverNotifications((-1), RTR(false), ReceivedFloatVal(1.0f){};

		void ReceivedHeartbeat(const int Device, int Val)
		{
			Serial.printf("Rcvd Heartbeat: %d from 0x%x\n", Val, Device);
			ReceivedID = CANID_HEARTBEAT;
		};

		void ReceivedHeartbeatRTR(const int Device)
		{
			Serial.printf("Received: RTR from 0x%x\n", Device);
			ReceivedID = CANID_HEARTBEAT;
			RTR = true;
		};
		
		void ReceivedVelocityQuad(const int Device, int DLC, int quadval)	// bitmask based on driver ID - mask = DEVID*DLC
		{
			Serial.printf("Rcvd quad: 0x%x from 0x%x w/ DLC=%d\n", quadval, Device, DLC);
			uint bitlength = DLC*2;	// length in bits of each number. data len = 8xDLC --> mask len = data len/4 = 2xDLC
			uint neg_mask = 1<<(bitlength-1);	
			int mask = (1<<bitlength)-1;
			int val = (quadval >> (bitlength*MyDeviceID)) & mask;	// extract int
			val = signExtend(val, bitlength);						// sign extend

			velocity_target = ThrottleToFloat(val) * speed_scale;
			Serial.printf("val+vel%d: %d\t %.2f ", MyDeviceID, val, velocity_target);
			Serial.print("\n");
			
			ReceivedID = CANID_VELOCITY_QUAD;
		};
		
		// void ReceivedVelocitySingle(const int Device, int Val)
		// {
		// 	Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
		// 	ReceivedID = CANID_VELOCITY_SINGLE;
		// };

		void ReceivedSpeedScale(const int Device, const float Val)
		{
			Serial.printf("Rcvd float: %.3f from 0x%x\n", Val, Device);
			ReceivedFloatVal = Val;
			ReceivedID = CANID_SPEED_SCALE;
		};

		// void ReceivedSFOCCmd(const int Device, const char* pText)
		// {
		// 	Serial.printf("Received: %s from 0x%x\n", pText, Device);
		// 	ReceivedID = CANID_SFOC;
		// };

		// void ReceivedMisc(const int Device, const char* pText)
		// {
		// 	Serial.printf("Received: %s from 0x%x\n", pText, Device);
 		// 	ReceivedID = CANID_MISC;
		// };

		// int ReceivedID;		
		// bool RTR = true;
		// float ReceivedFloatVal;
};


// Instatiation of the class which receives messages from the CAN bus.
// This class depends on your application!
RxFromCAN CANBroker;

// The actual CAN bus class, which handles all communication.
CANDriver CANDevice(CreateCanLib(pinTx, pinRx), &CANBroker);

#pragma endregion CANSetup

void setup() 
{
	// Serial1.begin(BAUDRATE);
	// Serial1.println("Serial Test");
	Serial.begin(BAUDRATE);
	SimpleFOCDebug::enable(&DebugSerial);
	delay(500);
	Serial.println("Started");
	// while (!Serial);

	delay(3000);


	pinMode(LED_BUILTIN, OUTPUT);
	CANDevice.Init();
	Serial.println("finished init"); delay(200);
	// Set bus termination on/off (may not be available on all platforms).
	if (CAN_OK!=CANDevice.Can1->SetBusTermination(true))
		Serial.println("Setting CAN bus termination via software not possible");
	Serial.println("finished termination"); delay(200);

	Serial.printf("CAN Setup complete; device ID is %d\n", MyDeviceID);

	#pragma region SimpleFOC_init
	Wire = Wire2;
	Wire.begin();
	delay(500);
	SIMPLEFOC_DEBUG("setck");
	Wire.setClock(400000);  // STM arduino platform setClock assumes Wire is initialized
	
	sensor.init();
	motor.linkSensor(&sensor);

	driver.voltage_power_supply = SUPPLY_VOLTAGE;
	// driver.voltage_limit = 9;  // optional driver output voltage limit
	driver.init();

	motor.linkDriver(&driver);

	// #DEBUG voltage limit
	motor.voltage_limit = 4;  // optional motor voltage limit - default = to driver limit
 	// motor.velocity_limit = 20; // default 20 rad/s
	
	// Tuned at 9V with 4V limit
	motor.PID_velocity.P = 0.12;
	motor.PID_velocity.I = 3;
	motor.PID_velocity.D = 0;

	// motor.LPF_velocity.Tf = 0.005;  // default 5ms; lower -> higher cutoff freq

	// choose FOC modulation
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	// set torque mode
	motor.torque_controller = TorqueControlType::voltage;
	// set motion control loop to be used
	motor.controller = MotionControlType::velocity;

	motor.useMonitoring(Serial);	// DEBUG FOC Motor monitoring

	// initialize motor
	motor.init();
	// align sensor and start FOC
	motor.initFOC();
	
	#ifdef COMMANDER
	command.add('M',doMotor,"my motor");
	command.add('O', toggle_monitoring, "toggle monitoring");
	command.add('D', mdisable, "disable motor");
	command.add('E', menable, "enable motor");
	command.add('T', setTarget, "set velocity target");
	#endif

	motor.disable();

	SIMPLEFOC_DEBUG("Setup Complete");
	#pragma endregion SimpleFOC_init
	_delay(1000);
}

uint32_t last_blink;
uint32_t bus_checkup;
void loop()
{
	
	motor.loopFOC();    // main FOC algorithm
		
	motor.move(velocity_target);   // FOC Motion control
  
	if (monitoring_var){motor.monitor();}

#ifdef COMMANDER
	command.run();  // commander interface
#endif

	if ((millis() - last_blink) > ledstatus){
		digitalWrite(PC11, ledstatus);
		ledstatus = !ledstatus;
		last_blink = millis();
	}

	#ifdef ARDUINO_GENERIC_G431CBUX
	static bool broadcasting = true;
	#else
	static bool broadcasting;
	#endif
	static bool silent;

	if (millis() - bus_checkup > 5000)
	{
		// Get some statistics on bus errors.
		static int LastTxErrors=0;
		static int LastRxErrors=0;
		static int LastOtherErrors = 0;
		static uint32_t LastStatus = 0;
		uint32_t Status = 0;
		char StatusStr[MAX_STATUS_STR_LEN]={0};

		CANDevice.Can1->GetStatus(&Status, StatusStr);
		if (CANDevice.Can1->GetTxErrors()!=LastTxErrors || CANDevice.Can1->GetRxErrors()!=LastRxErrors || CANDevice.Can1->GetOtherErrors()!=LastOtherErrors || LastStatus!=Status)
		{
			LastTxErrors = CANDevice.Can1->GetTxErrors();
			LastRxErrors = CANDevice.Can1->GetRxErrors();
			LastOtherErrors = CANDevice.Can1->GetOtherErrors();
			LastStatus = Status;

			Serial.printf("\nNew Status=%s, RxErrors=%d, TxErrors=%d, Other=%d\n", StatusStr, LastTxErrors, LastRxErrors, LastOtherErrors);
		}
	}

	if (millis() - last_blink > led_time){
		ledstatus = !ledstatus;
		last_blink = millis();
		digitalWrite(LED_BUILTIN, ledstatus);
	}

	//#DEBUG Serial console interface
	#ifdef DEBUG
	if (Serial.available())
		{
			uint64_t quad;
			String s;
			char c;
			s = Serial.readString();
			c = s.charAt(0);
			switch(c)
			{
				case '!':
					
					// #TODO: Replace all the printing with debug prints
					#ifdef ARDUINO_GENERIC_G431CBUX
					Serial.print("Generic G431CBU - ");
					#elif defined(STM32G4xx)
					Serial.print("STM32G4xx - ");
					#elif defined(PICO)
					Serial.print("PICO - ");
					#endif
					Serial.printf("Device ID is %d ; 0x%x ; ", MyDeviceID, MyDeviceID);
					Serial.println(MyDeviceID, BIN);
					Serial.print("STM UinqueID8 is: ");
					for(size_t i = 0; i < 8; i++)
  					{
						Serial.print(UniqueID8[i], HEX);
						Serial.print(" ");
					}
					Serial.println();
					Serial.println("Message IDs are:");
					Serial.print("H HEARTBEAT: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					Serial.print("R HEARTBEAT RTR: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					Serial.print("Q VELOCITY QUAD: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
					Serial.print("V VELOCITY SINGLE: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
					Serial.print("C SPEED SCALE (float): "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
					Serial.print("S SFOC: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
					Serial.print("M MISC: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
					Serial.println("Type any non-cmd character for command options.");
					break;
				case 'H':
					Serial.printf("Sending Heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					CANDevice.CANSendInt(1, CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					break;
				case 'R':
					Serial.printf("Request heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					CANDevice.RequestHeartbeat(MyDeviceID);
					break;
				case 'Q':
					Serial.println("Sending Velocity Quad (-127 to 127) for 32b");
					CANBroker.RTR=false;
					CANDevice.SendVelocityQuad(127, 125, 127, 125, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
					// quad = 0b01111111 << 24 | 0b10000001 <<16 | 0b01011101 <<8 | 0b10100011; 	// 127 -127 93 -93
					// // quad = (127 << 24) & (-127 << 16) & (93 << 8) & (-93);
					// CANDevice.CANSendInt(quad, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
					break;
				case 'V':
					Serial.println("Sending Velocity Single -528 ");
					CANDevice.CANSendInt(-528, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
					break;
				case 'C':
					Serial.printf("Sending Speed Scale: %.3f (ID 0x%x)\n", 1.0, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));		//#TODO: 
					CANDevice.CANSendFloat(speed_scale, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
					break;
				case 'S':
					Serial.printf("Sending SFOC (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
					CANDevice.CANSendText("Pong", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
					break;
				case 'G':
					Serial.printf("Sending Misc. (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
					CANDevice.CANSendText("Misc. Msg", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
					break;
				case 'I':
					MyDeviceID++;
					if (MyDeviceID > 5) MyDeviceID = 1;
					Serial.printf("Bumped ID - %d\n", MyDeviceID);
					break;
				default:
					Serial.printf("Options:\n! - list ID info\nP - Send Ping\nO - Send Pong\nI - Send Int value\nC - Send float Speed Scale %f\nR - Send int request (should get 1234 back)\nB - Toggle broadcasting\nS - Toggle silence\nD - Bump ID", speed_scale);
			}
		}
	#endif
	// Update message queues.
	CANDevice.Can1->Loop();
}

#endif