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
#define CD_MAKE_CAN_ID(Device, Message)     ((Message<<4) | CanID) 
#define CD_GET_MESSAGE_ID(CanID)            (CanID >> 4)	// 
#define CD_GET_DEVICE_ID(CanID)             (CanID & 0xf)      // Max ID = 127!

class CANDriverNotifications {
    public:
        virtual void ReceivedHeartbeat(const int Device, int val)=0;
		  virtual void ReceivedHeartbeatRTR(const int Device)=0;
        virtual void ReceivedVelocityQuad(const int Device, int val)=0;			// bitmask based on driver ID - mask = DEVID*DLC
		  virtual void ReceivedVelocitySingle(const int Device, int val)=0;
        virtual void ReceivedSpeedScale(const int Device, float val)=0;
        virtual void ReceivedSFOCCmd(const int Device, const char* pText)=0;
		  virtual void ReceivedMisc(const int Device, const char* pText)=0;
      //   virtual void ReceivedRequestInt(const int Device)=0;
};


class CANDriver : public SimpleCANProfile {
	public:
		CANDriver(SimpleCan* pCan, CANDriverNotifications* _pRxCommands) : SimpleCANProfile(pCan)
		{
			pRxCommands = _pRxCommands;
		}

	void  RequestHeartbeat(int DeviceID)
		{
			Can1->RequestMessage(2, CD_MAKE_CAN_ID(DeviceID, CANID_HEARTBEAT));            
		}

		void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
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
				case CANID_HEARTBEAT:
					if (rxHeader.RxFrameType==CAN_REMOTE_FRAME)
						pRxCommands->ReceivedHeartbeatRTR(Device);
					else
					{
						int val = CANGetInt(rxData);
						pRxCommands->ReceivedHeartbeat(Device, val);
					}
					break;
				case CANID_VELOCITY_QUAD:
					int val = CANGetInt(rxData);
					pRxCommands->ReceivedVelocityQuad(Device, val);
					break;
				case CANID_VELOCITY_SINGLE:
					int val = CANGetInt(rxData);
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
		}

	private:
		CANDriverNotifications* pRxCommands;
}