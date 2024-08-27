#ifndef CANDRIVER_CPP
#define CANDRIVER_CPP
/*
CAN-Bus controlled driver
*/
#include <Arduino.h>
#include <math.h>

#ifdef PICO
#warning PICO - 17-CANTX 16-CANRX 
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
			Serial.printf("Rcvd quad: 0x%x from 0x%x\n", quadval, Device);
			uint bitlength = DLC*2;	// length in bits of each number. data len = 8xDLC --> mask len = data len/4 = 2xDLC
			uint neg_mask = 1<<(bitlength-1);	
			int mask = (1<<bitlength)-1;
			int val = (quadval >> (bitlength*MyDeviceID)) & mask;	// extract int
			val = signExtend(val, bitlength);						// sign extend
			Serial.printf("val%d: %d ", MyDeviceID, val);
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

// HardwareSerial Serial3(PB11, PB10);   // uart3
// HardwareSerial Serial1(PB7, PB6);  // uart1
bool ledstatus;
uint32_t led_time = 0;
void setup() 
{
	// Serial1.begin(BAUDRATE);
	// Serial1.println("Serial Test");
	Serial.begin(BAUDRATE);
	delay(500);
	Serial.println("Started");
	// while (!Serial);

	delay(3000);

	uint8_t pinRx, pinTx;

	pinMode(LED_BUILTIN, OUTPUT);
	CANDevice.Init();
	Serial.println("finished init"); delay(200);
	// Set bus termination on/off (may not be available on all platforms).
	if (CAN_OK!=CANDevice.Can1->SetBusTermination(true))
		Serial.println("Setting CAN bus termination via software not possible");
	Serial.println("finished termination"); delay(200);

	Serial.printf("Setup done, device ID is %d\n", MyDeviceID);
}

void loop()
{
	static uint32_t LastAction=millis();
	static uint32_t LastFloatAction=millis();
	static uint32_t LastRTR=millis();
	#ifdef ARDUINO_GENERIC_G431CBUX
	static bool broadcasting = true;
	#else
	static bool broadcasting;
	#endif
	static bool silent;

	// Get some statistics on bus errors.
	static int LastTxErrors=0;
	static int LastRxErrors=0;
	static int LastOtherErrors = 0;
	static uint32_t LastStatus = 0;
	uint32_t Status = 0;
	char StatusStr[MAX_STATUS_STR_LEN]={0};
	#define FLOATVAL 4.02

	CANDevice.Can1->GetStatus(&Status, StatusStr);
	if (CANDevice.Can1->GetTxErrors()!=LastTxErrors || CANDevice.Can1->GetRxErrors()!=LastRxErrors || CANDevice.Can1->GetOtherErrors()!=LastOtherErrors || LastStatus!=Status)
	{
		LastTxErrors = CANDevice.Can1->GetTxErrors();
		LastRxErrors = CANDevice.Can1->GetRxErrors();
		LastOtherErrors = CANDevice.Can1->GetOtherErrors();
		LastStatus = Status;

		Serial.printf("\nNew Status=%s, RxErrors=%d, TxErrors=%d, Other=%d\n", StatusStr, LastTxErrors, LastRxErrors, LastOtherErrors);
	}

	if (millis() - led_time > 200){
		ledstatus = !ledstatus;
		led_time = millis();
		digitalWrite(LED_BUILTIN, ledstatus);
	}

	// serial feedback
	if (Serial.available())
		{
			uint64_t quad;
			String s;
			char c;
			s = Serial.readString();
			c = s.charAt(0);
			switch(c)
			{
				case '?':
					
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
					Serial.println("Message IDs are:");
					Serial.print("H HEARTBEAT: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					Serial.print("R HEARTBEAT RTR: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
					Serial.print("V VELOCITY QUAD: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
					Serial.print("E VELOCITY SINGLE: "); Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE), BIN); Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
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
				case 'V':
					Serial.println("Sending Velocity Quad (-127 to 127) for 32b");
					CANBroker.RTR=false;
					quad = 0b01111111 << 24 | 0b10000001 <<16 | 0b01011101 <<8 | 0b10100011; 	// 127 -127 93 -93
					// quad = (127 << 24) & (-127 << 16) & (93 << 8) & (-93);
					CANDevice.CANSendInt(quad, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
					break;
				case 'E':
					Serial.println("Sending Velocity Single -528 ");
					CANDevice.CANSendInt(-528, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
					break;
				case 'C':
					Serial.printf("Sending Speed Scale: %.3f (ID 0x%x)\n", FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
					CANDevice.CANSendFloat(FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
					break;
				case 'S':
					Serial.printf("Sending SFOC (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
					CANDevice.CANSendText("Pong", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
					break;
				case 'M':
					Serial.printf("Sending Misc. (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
					CANDevice.CANSendText("Misc. Msg", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
					break;
				case 'D':
					MyDeviceID++;
					if (MyDeviceID > 5) MyDeviceID = 1;
					Serial.printf("Bumped ID - %d\n", MyDeviceID);
					break;
				default:
					Serial.printf("Options:\n? - list ID info\nP - Send Ping\nO - Send Pong\nI - Send Int value\nF - Send float value %f\nR - Send int request (should get 1234 back)\nB - Toggle broadcasting\nS - Toggle silence\nD - Bump ID", FLOATVAL);
			}
		}

	// Update message queues.
	CANDevice.Can1->Loop();
}

#endif