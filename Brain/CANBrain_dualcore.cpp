/*
 *** CANDriver - Brain ***
 Dual-core controller for RPi =Serial=> Brain =CAN=> Drivers comms

 Core 0:
  Serial Comms + main rover state
 Core 1:
  CAN Comms
*/

#ifndef CANDRIVER_CPP
#define CANDRIVER_CPP

#ifdef DEBUG
#  define D(x) x
#else
#  define D(x)
#endif // Debug macro

#include <Arduino.h>
#include <math.h>
#include "ControlInterface.h"

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

int MyDeviceID = DEVICEID;

#include "SimpleCAN.h"
#include "CANDriverProfile.h"
#include "pico/util/queue.h"
#warning USING QUEUES

// Function to sign-extend an integer up to 32 bits
int32_t signExtend(int32_t value, int bitLength)
{
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

	void ReceivedVelocityQuad(const int Device, int DLC, int quadval) // bitmask based on driver ID - mask = DEVID*DLC
	{
		Serial.printf("Rcvd quad: 0x%x from 0x%x\n", quadval, Device);
		uint bitlength = DLC * 2; // length in bits of each number. data len = 8xDLC --> mask len = data len/4 = 2xDLC
		uint neg_mask = 1 << (bitlength - 1);
		int mask = (1 << bitlength) - 1;
		int val;

		// int r = signextend<signed int,5>(x);  // sign extend 5 bit number x to r
		for (int rxdevice = 0; rxdevice < 4; rxdevice++)
		{
			val = (quadval >> (bitlength * rxdevice)) & mask; // extract int
			val = signExtend(val, bitlength);				  // sign extend
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
RxFromCAN CANBroker;

// The actual CAN bus class, which handles all communication.
CANDriver CANDevice(CreateCanLib(pinTx, pinRx), &CANBroker);

// Single char items containing instructions on what to do
queue_t sendToCAN;		// [ D->Disable motors/drivers | E->Enable motors/drivers | C->Process & send SFOC Commander msg | S->Process string message ] (C -> Command packet received? maybe not...)
queue_t receivedCAN;

bool ledstatus;
uint32_t led_time = 0;



// Loop 0: Pi <-> Pico Comms
// Loop 1: Pico <-> Driver Comms
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
	Serial.println("finished init");
	delay(200);
	// Set bus termination on/off (may not be available on all platforms).
	if (CAN_OK != CANDevice.Can1->SetBusTermination(true))
		Serial.println("Setting CAN bus termination via software not possible");
	Serial.println("finished termination");
	delay(200);

	Serial.printf("Setup done, device ID is %d\n", MyDeviceID);
}

#ifdef ARDUINO_GENERIC_G431CBUX
	static bool broadcasting = true;
#else
	static bool broadcasting;
#endif
	static bool silent;

#ifndef TEST_CANMSG // end simple CAN Msg Test

MsgPack::Unpacker unpacker;
MsgPack::Packer packer;

ControlPacket cmd = {false, false, 125, 126, "K"};
char cbuff[64];
char c = '\0';

bool blinkstate;
bool read_success;
byte msgdata[128]; // #TODO: see if this is enough?
u8_t i = 0;
u8_t j = 0;
char lenbuf[5];     // up to 99999 bytes?
uint msglen = 0;
u8_t blinkduration = 200;
u32_t lastupdate = 0;
u32_t blinktime = 0;

/*  Main Loop - High-level Comms*/
void loop()
{
	// Read characters until '\n'. MsgPack messages will be preceeded by "\n~[msg_len]\n", then followed by the binary data
	while (Serial.available() && c != '\n' && i < 64)
	{
		c = Serial.read();
		memcpy(&cbuff[i], &c, 1);
		i++;
	}
	// Received data
	if (i > 0)
	{
		blinkduration = (blinkduration == 200 ? 500 : 200);

		// Received String - do something with it
		D(Serial.printf("-\n-Received Message: '%s'\n",cbuff);)
		i = 0;
		if (c == '\n')
		{
			c = Serial.read();
			if (c == '~')
			{
				j = 0;
				c = Serial.read();
				while (c != '~' && j < 5)
				{
					lenbuf[j] = c;
					c = Serial.read();
					j++;
				}
				msglen = strtol(lenbuf, 0, 0);
				memset(lenbuf, 0, sizeof(lenbuf));

				if (!msglen)
					return;
				j = Serial.readBytes(msgdata, msglen);
				if (j < msglen)
				{
					Serial.printf("Incomplete Msgpack; waiting on %dB...", msglen - j);
					Serial.readBytes(msgdata, msglen - j);
				}
				//#DEBUG - data prints
				D(Serial.print("Data: ");)
				D(Serial.write(msgdata, msglen);)
				D(Serial.printf("\nLen: %d\n", msglen);)
				if (!unpacker.feed(msgdata, msglen))
					return;
				// if (unpacker.isArray()) // #TODO: define EXT array type for various control packets
				if (unpacker.deserialize(cmd))
				{
#ifdef ACKPACKET
					Serial.println("ACK\r");
#endif
					if (cmd.s != "")
					{
						// Process String message...
						c = 'S';
						queue_add_blocking(&sendToCAN, &c);
					}
#ifdef ECHO_MSGPACK
					PrintPacket(&cmd);
#endif
				}
				else
					Serial.println(F("\r\nFailed to deserialize!\r"));
			}
			else
			{
				// Message is a string; Reset and continue reading characters
				cbuff[i] = c;
				i++;
			}
		}
		else
		{
			// #TODO: Reached end of buffer while reading string; process part of string message
			Serial.print(F("\r\nReceived partial string with length "));
			Serial.println(i);
			i = 0;
		}
	}

	// Report most recent command
	if (millis() - lastupdate > 5000)
	{
		packer.serialize(cmd);
		Serial.print(PACKETDELIM);
		Serial.print(packer.size());
		Serial.print(LEN_SEP);
		Serial.write((char *)packer.data(), packer.size());
		packer.clear();
		Serial.println();
		lastupdate = millis();
	}
	if (millis() - blinktime > blinkduration)
	{
		digitalWrite(LED_BUILTIN, blinkstate);
		blinkstate = !blinkstate;
		blinktime = millis();
	}
}

void PrintPacket(ControlPacket *cmd)
{
	Serial.printf("CP: %d %d %d %d %s\n", cmd->a, cmd->b, cmd->ljx, cmd->ljy, cmd->s);
}
#else	// TEST_CANMSG
void loop()
{
	static uint32_t LastAction = millis();
	static uint32_t LastFloatAction = millis();
	static uint32_t LastRTR = millis();
#ifdef ARDUINO_GENERIC_G431CBUX
	static bool broadcasting = true;
#else
	static bool broadcasting;
#endif
	static bool silent;

	// Get some statistics on bus errors.
	static int LastTxErrors = 0;
	static int LastRxErrors = 0;
	static int LastOtherErrors = 0;
	static uint32_t LastStatus = 0;
	uint32_t Status = 0;
	char StatusStr[MAX_STATUS_STR_LEN] = {0};


	// serial feedback
	if (Serial.available())
	{
		uint64_t quad;
		String s;
		char c;
		s = Serial.readString();
		c = s.charAt(0);
		switch (c)
		{
		case '?':

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
			Serial.print("H HEARTBEAT: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
			Serial.print("R HEARTBEAT RTR: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
			Serial.print("V VELOCITY QUAD: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
			Serial.print("E VELOCITY SINGLE: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
			Serial.print("C SPEED SCALE (float): ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
			Serial.print("S SFOC: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
			Serial.print("M MISC: ");
			Serial.print(CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC), BIN);
			Serial.printf(" ; 0x%x\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
			Serial.println("Type any non-cmd character for command options.");
			break;
		case 'H':
			Serial.printf("Sending Heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
			// CANDevice.CANSendInt(1, CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
			queue_add_blocking(&sendToCAN, &c);
			break;
		case 'R':
			queue_add_blocking(&sendToCAN, &c);
			// CANDevice.RequestHeartbeat(MyDeviceID);
			Serial.printf("Request heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
			break;
		case 'V':
			queue_add_blocking(&sendToCAN, &c);
			Serial.println("Sending Velocity Quad (-127 to 127) for 32b");
			// CANBroker.RTR=false;
			quad = 0b01111111 << 24 | 0b10000001 << 16 | 0b01011101 << 8 | 0b10100011; // 127 -127 93 -93
			// quad = (127 << 24) & (-127 << 16) & (93 << 8) & (-93);
			// CANDevice.CANSendInt(quad, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
			break;
		case 'E':
			queue_add_blocking(&sendToCAN, &c);
			Serial.println("Sending Velocity Single -528 ");
			// CANDevice.CANSendInt(-528, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
			break;
		case 'C':
			queue_add_blocking(&sendToCAN, &c);
			Serial.printf("Sending Speed Scale: %.3f (ID 0x%x)\n", FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
			// CANDevice.CANSendFloat(FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
			break;
		case 'S':
			queue_add_blocking(&sendToCAN, &c);
			Serial.printf("Sending SFOC (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
			// CANDevice.CANSendText("Pong", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
			break;
		case 'M':
			queue_add_blocking(&sendToCAN, &c);
			Serial.printf("Sending Misc. (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
			// CANDevice.CANSendText("Misc. Msg", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
			break;
		// case 'D':
		// 	MyDeviceID++;
		// 	if (MyDeviceID > 5) MyDeviceID = 1;
		// 	Serial.printf("Bumped ID - %d\n", MyDeviceID);
		// 	break;
		default:
			Serial.printf("Options:\n? - list ID info\nP - Send Ping\nO - Send Pong\nI - Send Int value\nF - Send float value %f\nR - Send int request (should get 1234 back)\nB - Toggle broadcasting\nS - Toggle silence\nD - Bump ID", FLOATVAL);
		}
	}

	// // Update message queues.
	// CANDevice.Can1->Loop();
}
#endif // end simple CAN Msg Test


u8_t cmd_interval_ms = 10;	// 100Hz - #LATER: what's the max?

/* Rover State */
long lastcmd_ms = 0;
long heartbeat_timeouts[4];
MotionVector mvec;

#define FLOATVAL 4.02
void setup1()
{
	queue_init(&sendToCAN, sizeof(char), 16);
}
void loop1()
{
	char c;
	if (millis() - led_time > 200)
	{
		ledstatus = !ledstatus;
		led_time = millis();
		digitalWrite(LED_BUILTIN, ledstatus);
	}
	if (!queue_is_empty(&sendToCAN))
	{
		uint64_t quad;
		queue_remove_blocking(&sendToCAN, &c);
		switch (c)
		{
		// case 'H':
		// 	// Serial.printf("Sending Heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
		// 	CANDevice.CANSendInt(1, CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
		// 	break;
		// case 'R':
		// 	// Serial.printf("Request heartbeat (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_HEARTBEAT));
		// 	CANDevice.RequestHeartbeat(MyDeviceID);
		// 	break;
		// case 'Q':
		// 	// Serial.println("Sending Velocity Quad (-127 to 127) for 32b");
		// 	CANBroker.RTR = false;
		// 	quad = 0b01111111 << 24 | 0b10000001 << 16 | 0b01011101 << 8 | 0b10100011; // 127 -127 93 -93
		// 	// quad = (127 << 24) & (-127 << 16) & (93 << 8) & (-93);
		// 	CANDevice.CANSendInt(quad, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_QUAD));
		// 	break;
		// case 'E':
		// 	// Serial.println("Sending Velocity Single -528 ");
		// 	CANDevice.CANSendInt(-528, CD_MAKE_CAN_ID(MyDeviceID, CANID_VELOCITY_SINGLE));
		// 	break;
		case 'V':
			// Serial.printf("Sending Speed Scale: %.3f (ID 0x%x)\n", FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
			CANDevice.CANSendFloat(FLOATVAL, CD_MAKE_CAN_ID(MyDeviceID, CANID_SPEED_SCALE));
			break;
		case 'C':
			// Serial.printf("Sending SFOC CMD (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
			CANDevice.CANSendText("Pong", CD_MAKE_CAN_ID(MyDeviceID, CANID_SFOC));
			break;
		case 'M':
			// Serial.printf("Sending Misc. (ID 0x%x)\n", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
			CANDevice.CANSendText("Misc. Msg", CD_MAKE_CAN_ID(MyDeviceID, CANID_MISC));
			break;
		// default:
		// Serial.printf("Options:\n? - list ID info\nP - Send Ping\nO - Send Pong\nI - Send Int value\nF - Send float value %f\nR - Send int request (should get 1234 back)\nB - Toggle broadcasting\nS - Toggle silence\nD - Bump ID", FLOATVAL);
		}
	}
	if (millis() - lastcmd_ms > cmd_interval_ms)
		{
			CANDevice.SendVelocityQuad(mvec.vFL, mvec.vFL, mvec.vBL, mvec.vBR);
			// servoFL.setpos(mvec.aFL);
			// servoFR.setpos(mvec.aFR);
			// servoBL.setpos(mvec.aBL);
			// servoBR.setpos(mvec.aBR);
		}

	// Update message queues.
	CANDevice.Can1->Loop();
};

#endif