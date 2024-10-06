#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include <math.h>
#include <MsgPack.h>

#ifdef CONTROLTEST
extern void setup();
extern void loop();
#endif

#define PACKETDELIM "\n~"
#define LEN_SEP '~'

#define JOY_MAX 1024

struct ControlPacket
{
	bool a;
	bool b;
	int16_t rt;			// throttle
	int16_t ljx = 125;	// steer x
	int16_t ljy = 126;	// steer y
	MsgPack::str_t s = "Hello, Pi!";
	MSGPACK_DEFINE(a, b, rt, ljx, ljy, s); // conversion to array -> [a, b, ljx, ljy, s]
};

const float STEERANGLE_MAX_RAD = M_PI_4;
const u8_t STEERCTR_D_MIN = 200;  //#FIXME steer center dmin value
const float STEERCTR_SCALING = 1;

/* Steer Center DX/DY (abs. distance from origin to wheels)*/
const u8_t SCDX = 114;
const u8_t SCDY = 141;
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

struct MotionVector
{
	int vFL;
	int vFR;
	int vBL;
	int vBR;
	int aFL;
	int aFR;
	int aBL;
	int aBR;
	MSGPACK_DEFINE(vFL, vFR, vBL, vBR, aFL, aFR, aBL, aBR);
};



void PrintPacket(ControlPacket *cmd);
void PrintStrMsg(char *msg, uint strlen, bool partial, bool overflow);

void CalcSteerCenter(int16_t *d, int16_t *h, int joyx, int joyy);
void CalcMotionVector(MotionVector *mvec, int joyx, int joyy, int throttle);

// MotionVector ProcessPacket(ControlPacket);

// Callback function which should be user supplied and which will be called for each ControlPacket
typedef void(*ControlPacketCallback)(ControlPacket *cmd);

// Callback function for received strings; partial (unterminated message) and overflow (ran out of buffer) are mutually exclusive
typedef void(*StrPacketCallback)(char *msg, uint strlen, bool partial, bool overflow);


class PacketHandler
{
	public:
		PacketHandler(const ControlPacketCallback cmdcb, const StrPacketCallback strcb);   // #TODO: pass serial instance
		// ~MsgPackHandler();   // Necessary if dynamic allocation
		void SetCmdCallback(const ControlPacketCallback callback);
		void SetStrCallback(const StrPacketCallback callback);
		ControlPacketCallback CmdCallback;
		StrPacketCallback StrCallback;
		
		int ParseSerial(u8_t cbuff_start_idx=0);
		void SendPacket(const ControlPacket *cmd);
		
		MsgPack::Unpacker unpacker;
		MsgPack::Packer packer;

		ControlPacket cmd = {false, false, 0, 125, 126, "Hello, World!"}; // #TODO: move to class member
		ControlPacket prevCmd;
	private:
		byte msgdata[128]; // #TODO: see if this is enough?
		char cbuff[64];

};

// Not Used Yet
struct RoverState
{
	MotionVector target_vel;
	// uint8_t motion_setpoint;
	
};

#endif