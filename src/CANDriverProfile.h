#include "SimpleCAN.h"

#define CANID_HEARTBEAT    1  //	RTR used to request detailed hearbeat/status from driver or controller. #LATER: determine contents and DLC
// #define CANID_RTRHEARTBEAT 2 	// 
#define CANID_VELOCITY_QUAD    4   //
#define CANID_VELOCITY_SINGLE   5   //
#define CANID_SPEED_SCALE   6       // Message is a floating point value that represents the scaling from {#TODO}-bit to {#TODO rad or cm}/s 
#define CANID_SFOC	7
// #define CANID_CD_RTRINT  8       // Request an int from the client
#define CANID_MISC  9       // Request an int from the client

// Device IDs
#define DEVID_CONTROLLER 0
#define DEVID_FL 1
#define DEVID_FR 2
#define DEVID_BL 3
#define DEVID_BR 4


// The CAN ID consists of the 7-bit mesage ID followed by the 4-bit device ID.
// The device ID is hard-coded into each controller, so arbitration is not currently handled.
#define CD_MAKE_CAN_ID(Device, Message)     ((Message<<4) | Device)
#define CD_GET_MESSAGE_ID(CanID)            (CanID >> 4)
#define CD_GET_DEVICE_ID(CanID)             (CanID & 0xf)      // Max ID = 127!

class CANDriverNotifications
{
	public:
		CANDriverNotifications() : ReceivedID(-1), RTR(false), ReceivedFloatVal(1.0f){};
		virtual void ReceivedHeartbeat(const int Device, int Val)=0;
		// Brain:	Refresh driver timeout;
		// Driver:	Refresh driver timeout (shorter than Brain's);

		virtual void ReceivedHeartbeatRTR(const int Device)=0;
		// Brain:	Send driver settings
		// Driver:	Report status
      
		virtual void ReceivedVelocityQuad(const int Device, int DLC, int quadval)=0;			// bitmask based on driver ID - mask = DEVID*DLC
		// Brain: ???
		// Driver: update current value... and ack?

		void ReceivedVelocitySingle(const int Device, int Val);
		virtual void ReceivedSpeedScale(const int Device, float Val)=0;
		// Brain:	feed back the speed scale to everyone...? or just dont filter it out? Maybe use special ID bit for broadcast
		// Driver:	update current speed scale... and ack?

		void ReceivedSFOCCmd(const int Device, const char* pText);	//#LATER
		// Brain:	TBD? compare motor config with expected config?
		// Driver:	apply config
		
		void ReceivedMisc(const int Device, const char* pText);	//#LATER
		// Brain:	store received info
		// Driver:	respond with requested info

		int ReceivedID;
		bool RTR = true;
		float ReceivedFloatVal;
};

void CANDriverNotifications::ReceivedVelocitySingle(const int Device, int Val)
		{
			Serial.printf("Rcvd int: %d from 0x%x\n", Val, Device);
			ReceivedID = CANID_VELOCITY_SINGLE;
		};
		
// void ReceivedSpeedScale(const int Device, const float Val)
// {
// 	Serial.printf("Rcvd float: %.3f from 0x%x\n", Val, Device);
// 	ReceivedFloatVal = Val;
// 	ReceivedID = CANID_SPEED_SCALE;
// };

void CANDriverNotifications::ReceivedSFOCCmd(const int Device, const char* pText)
{
	Serial.printf("Received: %s from 0x%x\n", pText, Device);
	ReceivedID = CANID_SFOC;
};

void CANDriverNotifications::ReceivedMisc(const int Device, const char* pText)
{
	Serial.printf("Received: %s from 0x%x\n", pText, Device);
	ReceivedID = CANID_MISC;
};


class CANDriver : public SimpleCANProfile {
	public:
		CANDriver(SimpleCan* pCan, CANDriverNotifications* _pRxCommands) : SimpleCANProfile(pCan)
		{
			pRxCommands = _pRxCommands;
		}

		void RequestHeartbeat(int DeviceID)
		{
			Can1->RequestMessage(2, CD_MAKE_CAN_ID(DeviceID, CANID_HEARTBEAT));            
		}

		void SendVelocityQuad(const int8_t vFL, const int8_t vFR, const int8_t vBL, const int8_t vBR);

		void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData);

	private:
		CANDriverNotifications* pRxCommands;
};

void CANDriver::SendVelocityQuad(const int8_t vFL, const int8_t vFR, const int8_t vBL, const int8_t vBR)	// IDs 1, 2, 3, 4
{
	u32_t quad = vFL | vFR << 8 | vBL << 16 | vBR << 24; // 127 -127 93 -93
};

void CANDriver::HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
{
	// Serial.println("@");

	#define MAX_STRLEN  16
	
	char Str[MAX_STRLEN];
	// float val=0;
	int Device = CD_GET_DEVICE_ID(rxHeader.Identifier);
	// char buf [32];
	// char buf2 [64];
	// utoa(rxHeader.Identifier,buf,2);
	// // BitStr(rxHeader.Identifier, buf, 32, (Msg.EFF ? 29 : 11), ' ');	// excess bits generated from utoa for some reason? extra '100' at the MSB 

	// memcpy(&val, rxData, rxHeader.DataLength);
	// // utoa(val, buf2,2);   // Data conversion to string - this doesn't work for some reason
	// // Serial.printf("R~ID:%32s (0x%x) DLC=%d Remote?%d EFF?%d\n", buf, rxHeader.Identifier, rxHeader.DataLength, rxHeader.RxFrameType, rxHeader.IdType);
	// // Serial.printf("\nR~ID:%32s (0x%x) DLC=%d Remote?%d EFF?%d\n data=%64s\n", buf, rxHeader.Identifier, rxHeader.DataLength, rxHeader.RxFrameType, rxHeader.IdType, (rxHeader.RxFrameType ? "~" : buf2)); //rxHeader.RxFrameType ? "~": buf2);
	switch(CD_GET_MESSAGE_ID(rxHeader.Identifier))
	{
		int val;
		case CANID_HEARTBEAT:
			if (rxHeader.RxFrameType==CAN_REMOTE_FRAME)
				pRxCommands->ReceivedHeartbeatRTR(Device);
			else
			{
				val = CANGetInt(rxData);
				pRxCommands->ReceivedHeartbeat(Device, val);
			}
			break;
		case CANID_VELOCITY_QUAD:
			val = CANGetInt(rxData);
			pRxCommands->ReceivedVelocityQuad(Device, rxHeader.DataLength, val);
			break;
		case CANID_VELOCITY_SINGLE:
			val = CANGetInt(rxData);
			pRxCommands->ReceivedVelocitySingle(Device, val);
			break;
		case CANID_SPEED_SCALE:
			val = CANGetFloat(rxData);
			pRxCommands->ReceivedSpeedScale(Device, val);
			break;
		case CANID_SFOC:
			CANGetString(rxData, Str, min(MAX_STRLEN, (int)rxHeader.DataLength));
			pRxCommands->ReceivedSFOCCmd(Device, Str);
			break;
		case CANID_MISC:
			CANGetString(rxData, Str, min(MAX_STRLEN-1, (int)rxHeader.DataLength));
			pRxCommands->ReceivedMisc(Device, Str);
			break;
		default:
			val = CANGetInt(rxData);

			char buf [32];
			char buf2 [64];

			utoa(rxHeader.Identifier,buf,2);
			utoa(val,buf2,2);

			Serial.printf("R~ID:%32s DLC=%d Remote?%d EFF?%d\n data=%64s\n\n", buf, rxHeader.DataLength, rxHeader.RxFrameType, rxHeader.IdType, (rxHeader.RxFrameType ? "~" : buf2));
	}
};