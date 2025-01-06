#ifndef CANDRIVER_CPP
#define CANDRIVER_CPP
/*

    Demo code for portable CAN bus library
   
	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/
#include <Arduino.h>
#include <math.h>
#include "ControlInterface.h"
#include "MotionControl.h"
#include "rover_config.h"

#ifdef PICO
#pragma message("PICO - 17-CANTX 16-CANRX")
uint8_t pinTx = 17;
uint8_t pinRx = 16;
#else
uint8_t pinTx = -1;
uint8_t pinRx = -1;
#endif

#ifndef DEVICEID
	#ifdef PICO
	#define DEVICEID 1
	#else
	#define DEVICEID 0
	#endif
#endif

int MyDeviceID=DEVICEID;

#include "SimpleCAN.h"
#include "CANDriverProfile.h"


//#TODO: Servo Pins

#pragma region SimpleCAN_Setup
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
			Serial.printf("Rcvd quad: 0x%x from 0x%x w/ DLC %d\n", quadval, Device, DLC);
			uint bitlength = DLC*2;	// length in bits of each number. data len = 8xDLC --> mask len = data len/4 = 2xDLC
			uint neg_mask = 1<<(bitlength-1);	
			int mask = (1<<bitlength)-1;
			int val;
			
			// int r = signextend<signed int,5>(x);  // sign extend 5 bit number x to r
			for (int rxdevice=0; rxdevice<4; rxdevice++)
			{
				val = (quadval >> (bitlength*rxdevice)) & mask;	// extract int
				val = signExtend(val, bitlength);				// sign extend
				Serial.printf("val%d: %d ", rxdevice, val);
			}
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

#pragma endregion SimpleCAN_Setup


ControlPacket cmd = {false, false, 0, 125, 126, "K"};
ControlPacket prevCmd = {};

// Rover State - Motion control
RoverState roverState;

void handlePacket(RoverState *rover, const ControlPacket *cmd);
PacketHandler handler([](ControlPacket *cmd){ handlePacket(&roverState, cmd); },
				  PrintStrMsg);


bool ledstatus;
uint32_t led_time = 0;
uint32_t status_time = 0;
void setup() 
{
	// Serial1.begin(BAUDRATE);
	// Serial1.println("Serial Test");
	Serial.begin(BAUDRATE);
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

	Serial.printf("Setup done, device ID is %d\n", MyDeviceID);

	// #TODO: Driver Initialization
	// roverState.initServos();	//FIXME: Servo initialization

	// Roll call

	// #TODO: RoverState Init throttle setting, direction, etc.
}


int parseResult;
int counter;
long motion_ping;


void loop()
{
	// TODO: Add watchdog to MotionControl RoverState class.. or some comms class	
	// static uint32_t LastAction=millis();
	// static uint32_t LastFloatAction=millis();
	// static uint32_t LastRTR=millis();

	// Set default state for broadcasting or sending messages
	#ifdef ARDUINO_GENERIC_G431CBUX
	static bool broadcasting = true;
	#else
	static bool broadcasting;
	#endif
	static bool silent;

	parseResult = handler.ParseSerial(); // For any ControlPackets, updates mvec. Otherwise, print string msg

	if (parseResult < 0)
	{
		Serial.print(F("ERR - Failed parsing serial w/ code "));
		Serial.println(parseResult);
	}
	else if (parseResult == 1)
	{
		//TODO: Add check for only string msg parsed to avoid sending extra motion cmds
		
		CANDevice.SendVelocityQuad(roverState.driveVec.vFL, roverState.driveVec.vFR, roverState.driveVec.vBL, roverState.driveVec.vBR, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
		// roverState.CommandServos();	//FIXME
	}

	// Monitor CAN Bus state
	if (millis() - status_time > 5000)
	{
		// Get some statistics on bus errors.
		static int LastTxErrors=0;
		static int LastRxErrors=0;
		static int LastOtherErrors = 0;
		static uint32_t LastStatus = 0;
		uint32_t Status = 0;
		char StatusStr[MAX_STATUS_STR_LEN]={0};
		

		// Monitor CAN Bus for erros. Is this possible right now?
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
	
	// Blink status LED 
	if (millis() - led_time > 200){
		ledstatus = !ledstatus;
		led_time = millis();
		digitalWrite(LED_BUILTIN, ledstatus);
	}

	
	// Update message queues.
	CANDevice.Can1->Loop();

	if (millis() - motion_ping > 5000)
	{
		CANDevice.SendVelocityQuad(roverState.driveVec.vFL, roverState.driveVec.vFR, roverState.driveVec.vBL, roverState.driveVec.vBR, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
		// Serial.printf("Loops: %d\n",counter);
		// counter = 0;
		motion_ping = millis();
	}
	// counter++;
}

// #FIXME: implement targetVec
/// @brief Calculates the motion vector, updating the rover state's targetVec
/// @param rover rover state 
/// @param cmd 
void handlePacket(RoverState *rover, const ControlPacket *cmd)
{
	// Serial.printf("jx,jy,rt: %5d, %5d, %4d\n", cmd->ljx, cmd->ljy, cmd->rt);
	float d, h;
	CalcSteerCenter(&d, &h, cmd->ljx, cmd->ljy);
	// CalcMotionVector(&rover->targetVec, d, h, cmd->rt);
	CalcMotionVector(&rover->driveVec, d, h, cmd->rt);
	MotionVector veccp = rover->driveVec;	// just for printing in case the motion vector gets overwritten while printing
	rover->Update();	// Calc next motor values
	
	// Serial.printf("SC: (%4f,%4f)\n", d, h);
	// Serial.printf("FL:%5d | %4d  FR: %5d | %4d  BL:%5d | %4d  BR: %5d %4d\n", veccp.vFL, veccp.aFL, veccp.vFR, veccp.aFR, veccp.vBL, veccp.aBL, veccp.vBR, veccp.aBR);
	handler.prevCmd = *cmd;	// TODO: get rid of or encapsulate reference to handler
}

#endif