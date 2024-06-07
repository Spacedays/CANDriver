/*

    Demo code for portable CAN bus library
   
	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/
#include <Arduino.h>
#include <math.h>

#ifdef ARDUINO_GENERIC_G431CBUX
#warning GENERIC G431 - PA12-CANTX PA11-CANRX PC11-LED_BUILTIN 

#define LED_BUILTIN PC11
uint8_t pinTx = PA12;
uint8_t pinRx = PA11;
#elif defined(STM32G4xx)
#warning B-G431B-ESC1
uint8_t pinTx = A_CAN_TX;
uint8_t pinRx = A_CAN_RX;
#elif defined(PICO)
uint8_t pinTx = 17;
uint8_t pinRx = 16;
#else
uint8_t pinTx = -1;
uint8_t pinRx = -1;
#endif


#include "SimpleCAN.h"
#include "CANDriverProfile.h"


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
		RxFromCAN() : ReceivedID(-1), RTR(false) , ReceivedFloatVal(1.0f){};

		void ReceivedHeartbeat(const int Device, int Val)
		{
			Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
			ReceivedID = CANID_HEARTBEAT;
		};

		void ReceivedHeartbeatRTR(const int Device)
		{
			Serial.printf("Received: RTR from 0x%x\n", Device);
			ReceivedID = CANID_HEARTBEAT;
			RTR = true;
		};
		
		void ReceivedVelocityQuad(const int Device, int Val)
		{
			Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
			ReceivedID = CANID_VELOCITY_BROADCAST;
		};
		
		void ReceivedVelocitySingle(const int Device, int Val)
		{
			Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
			ReceivedID = CANID_VELOCITY_SINGLE;
		};

		void ReceivedSpeedScale(const int Device, const float Val)
		{
			Serial.printf("Rcvd float: %.3f from 0x%x\n", Val, Device);
			ReceivedFloatVal = Val;
			ReceivedID = CANID_SPEED_SCALE;
		};

		void ReceivedSFOCCmd(const int Device, const char* pText)
		{
			Serial.printf("Received: %s from 0x%x\n", pText, Device);
			ReceivedID = CANID_SFOC;
		};

		void ReceivedMisc(const int Device, const char* pText)
		{
			Serial.printf("Received: %s from 0x%x\n", pText, Device);
 			ReceivedID = CANID_MISC;
		};

		int ReceivedID;		
		bool RTR = true;
		float ReceivedFloatVal;
};



// Instatiation of the class which receives messages from the CAN bus.
// This class depends on your application!
RxFromCAN CANBroker;

// The actual CAN bus class, which handles all communication.
CANPingPong CANDevice(CreateCanLib(pinTx, pinRx), &CANBroker);


int MyDeviceID=0;
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
	// Create a random device ID to avoid conflicts on the CAN bus.
	#ifdef ARDUINO_GENERIC_G431CBUX
	MyDeviceID = analogRead(PC4) & 127;
	#define LED_BUILTIN PC11
	#elif defined(STM32G4xx)
	//B-G431B-ESC1
	MyDeviceID = analogRead(A_POTENTIOMETER) & 127;
	#elif defined(PICO)
	MyDeviceID = analogRead(26) & 127;
	#else
	MyDeviceID = random(1, 127);
	#endif

	pinMode(LED_BUILTIN, OUTPUT);
	CANDevice.Init();
	Serial.println("finished init"); delay(200);
	// Set bus termination on/off (may not be available on all platforms).
	if (CAN_OK!=CANDevice.Can1->SetBusTermination(true))
		Serial.println("Setting CAN bus termination via software not possible");
	Serial.println("finished termination"); delay(200);

	Serial.printf("Setup done, random device ID is %d\n", MyDeviceID);
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

	int RandWait = random(-500, 1000);

	// Test of regular messages:
	// What is sent next to the CAN bus depends on what was received last. 
	// When a PING was received, send a PONG and vice versa.
	// To get the whole thing started, a PONG is sent every 5s without having received anything.
	// This is just for testing. Usually you would invoke actions for incomming messages
	// directly in the broker class.
	if (!silent) {	
		if ((CANBroker.ReceivedID==CANID_PP_PING && LastAction+1000<millis()) || (LastAction+RandWait+7000<millis() && broadcasting) )
		{
			Serial.printf("!Sending Pong (%x)\n",PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
			CANDevice.CANSendText("Pong", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
			LastAction=millis();

			// Make sure we don't react twice to the same message.
			CANBroker.ReceivedID = -1;
		}
		else if (CANBroker.ReceivedID==CANID_PP_PONG && LastAction+RandWait+1000<millis())
		{
			Serial.printf("!Sending Ping (%x)\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
			CANDevice.CANSendText("Ping", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
			LastAction=millis();

			// Make sure we don't react twice to the same message.
			CANBroker.ReceivedID = -1;		
		}
		else if (CANBroker.ReceivedID==CANID_PP_RTRINT && CANBroker.RTR)
		{
			Serial.println("!Sending int 1234");
			// React to an RTR request message. The reply should be the number "1234". If something else is 
			// received, check the byte order used by the devices!
			CANBroker.RTR=false;
			CANDevice.CANSendInt(1234, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT));

			// Make sure we don't react twice to the same message.
			CANBroker.ReceivedID = -1;		
		}


		// Every 8s just send a float value. This can be used to check if all devices on 
		// the bus use the same floating point number representation and byte order.
		if (LastFloatAction+RandWait+8000<millis() && broadcasting)
		{
			float NewVal = CANBroker.ReceivedFloatVal*2.5;
			if (NewVal==0) NewVal=1.0f;
			if (NewVal>1000000) NewVal=-1.0;
			if(NewVal<-1000000) NewVal = 1.0;

			Serial.printf("!SendingF: %.3f \n", NewVal);
			CANDevice.CANSendFloat(NewVal, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT));
			LastFloatAction=millis();
		}

		// Test of RTR messages
		// Every 10s request an int value. Response should be the number 1234 in binary form.
		if (LastRTR+RandWait+10000<millis() && broadcasting)
		{
			Serial.printf("!Request int\n");
			CANDevice.CANRequestInt(MyDeviceID);
			LastRTR=millis();
		}
	}

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
			String s;
			char c;
			s = Serial.readString();
			c = s.charAt(0);
			switch(c)
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
					Serial.print("PING: "); Serial.print(PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING), BIN); Serial.printf(" ; 0x%x\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
					Serial.print("PONG: "); Serial.print(PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG), BIN); Serial.printf(" ; 0x%x\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
					Serial.print("FLOAT: "); Serial.print(PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT), BIN); Serial.printf(" ; 0x%x\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT));
					Serial.print("RTRINT: "); Serial.print(PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT), BIN); Serial.printf(" ; 0x%x\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT));
					Serial.println("Type any non-cmd character for command options.");
					break;
				case 'P':
					Serial.printf("Sending Ping (ID 0x%x)\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
					CANDevice.CANSendText("Ping", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PING));
					break;
				case 'O':
					Serial.printf("Sending Pong (ID 0x%x)\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
					CANDevice.CANSendText("Pong", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_PONG));
					break;
				case 'I':
					Serial.println("Sending int 2222");
					CANBroker.RTR=false;
					CANDevice.CANSendInt(2222, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT));
					break;
				case 'F':
					Serial.printf("SendingF: %.3f (ID 0x%x)\n", FLOATVAL, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT));
					CANDevice.CANSendFloat(FLOATVAL, PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_FLOAT));
					break;
				case 'R':
					Serial.printf("Request int (ID 0x%x)\n", PP_MAKE_CAN_ID(MyDeviceID, CANID_PP_RTRINT));
					CANDevice.CANRequestInt(MyDeviceID);
					break;
				case 'B':
					broadcasting = !broadcasting;
					Serial.printf("Toggled Broadcasting - %d\n", broadcasting);
					break;
				case 'S':
					silent = !silent;
					Serial.printf("Toggled Silence - %d\n", silent);
					CANBroker.ReceivedID = -1;	// ignore response to last received message
					break;
				case 'D':
					MyDeviceID++;
					Serial.printf("Bumped ID - %d\n", MyDeviceID);
					break;
				default:
					Serial.printf("Options:\n? - list ID info\nP - Send Ping\nO - Send Pong\nI - Send Int value\nF - Send float value %f\nR - Send int request (should get 1234 back)\nB - Toggle broadcasting\nS - Toggle silence\nD - Bump ID", FLOATVAL);
			}
		}

	// Update message queues.
	CANDevice.Can1->Loop();
}
