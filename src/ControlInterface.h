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

const int16_t JOY_MAX = 32767;

struct ControlPacket
{
	bool a;
	bool b;
	int16_t rt;			 // throttle
	int16_t ljx = 125; // steer x
	int16_t ljy = 126; // steer y
	MsgPack::str_t s = "Hello, Pi!";
	MSGPACK_DEFINE(a, b, rt, ljx, ljy, s); // conversion to array -> [a, b, ljx, ljy, s]
};
template <typename T>
int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

// steering angles are stored as a delta wrt center position (which is 90 degrees)
const u8_t STEERCTR_D_MIN = 200; // #FIXME steer center dmin value
const u8_t STEERCTR_SCALING = 200;
const float STEERANGLE_MAX_RAD = M_PI_4;
const uint8_t STEER_RATIO = 2; // steering is REDUCED by this amount. (e.g. servo delta of 90 = steering delta of 45)

/* Steer Center DX/DY (abs. distance from origin to wheels)*/
const u8_t SCDX = 114;
const u8_t SCDY = 141;

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
bool operator==(const MotionVector &lhs, const MotionVector &rhs)
{
	return lhs.aFL == rhs.aFL && lhs.aFR == rhs.aFR &&
			 lhs.aBL == rhs.aBL && lhs.aBR == rhs.aBR &&
			 lhs.vFL == rhs.vFL && lhs.vFR == rhs.vFR &&
			 lhs.vBL == rhs.vBL && lhs.vBR == rhs.vBR;
}
MotionVector operator+(const MotionVector &lhs, const MotionVector &rhs)
{
	MotionVector mvec = {
		 lhs.aFL + rhs.aFL,
		 lhs.aFR + rhs.aFR,
		 lhs.aBL + rhs.aBL,
		 lhs.aBR + rhs.aBR,
		 lhs.vFL + rhs.vFL,
		 lhs.vFR + rhs.vFR,
		 lhs.vBL + rhs.vBL,
		 lhs.vBR + rhs.vBR,
	};
	return mvec;
}
MotionVector operator-(const MotionVector &lhs, const MotionVector &rhs)
{
	MotionVector mvec = {
		 lhs.aFL - rhs.aFL,
		 lhs.aFR - rhs.aFR,
		 lhs.aBL - rhs.aBL,
		 lhs.aBR - rhs.aBR,
		 lhs.vFL - rhs.vFL,
		 lhs.vFR - rhs.vFR,
		 lhs.vBL - rhs.vBL,
		 lhs.vBR - rhs.vBR,
	};
	return mvec;
}
// Full multiplication
MotionVector operator*(const MotionVector &lhs, const float mult)
{
	MotionVector mvec = {
		 lhs.aFL * mult,
		 lhs.aFR * mult,
		 lhs.aBL * mult,
		 lhs.aBR * mult,
		 lhs.vFL * mult,
		 lhs.vFR * mult,
		 lhs.vBL * mult,
		 lhs.vBR * mult,
	};
	return mvec;
}
// Angle multiplication
MotionVector operator%(const MotionVector &lhs, const float mult)
{
	MotionVector mvec = {
		 lhs.aFL * mult,
		 lhs.aFR * mult,
		 lhs.aBL * mult,
		 lhs.aBR * mult,
		 lhs.vFL,
		 lhs.vFR,
		 lhs.vBL,
		 lhs.vBR,
	};
	return mvec;
}

void PrintPacket(ControlPacket *cmd);
void PrintStrMsg(char *msg, uint strlen, bool partial, bool overflow);

void CalcSteerCenter(int16_t *d, int16_t *h, int joyx, int joyy);
void CalcMotionVector(MotionVector *mvec, ControlPacket *cmd);
void CalcMotionVector(MotionVector *mvec, int16_t joyx, int16_t joyy, int16_t throttle);

// MotionVector ProcessPacket(ControlPacket);

// Callback function which should be user supplied and which will be called for each ControlPacket
typedef void (*ControlPacketCallback)(ControlPacket *cmd);

// Callback function for received strings; partial (unterminated message) and overflow (ran out of buffer) are mutually exclusive
typedef void (*StrPacketCallback)(char *msg, uint strlen, bool partial, bool overflow);

// Parses MsgPack Packets, triggering ControlPacket and String callbacks as appropriate.
// The user-supplied CmdCallBack and StrCallback recieve a CommandPacket or string respectively
class PacketHandler
{
public:
	PacketHandler(const ControlPacketCallback cmdcb, const StrPacketCallback strcb); // #TODO: pass serial instance
	
	// ~MsgPackHandler();   // Necessary if dynamic allocation
	void SetCmdCallback(const ControlPacketCallback callback);
	void SetStrCallback(const StrPacketCallback callback);
	ControlPacketCallback CmdCallback;
	StrPacketCallback StrCallback;

	int ParseSerial(u8_t cbuff_start_idx = 0);
	void SendPacket(const ControlPacket *cmd);

	MsgPack::Unpacker unpacker;
	MsgPack::Packer packer;

	// #TODO: check if cmd and prevcmd are actually used anywhere
	ControlPacket cmd = {false, false, 0, 125, 126, "Hello, World!"};	// Used as a buffer to deserialize into that doesn't go in & out of scope every message.
	ControlPacket prevCmd =  {false, false, 0, 125, 126, "Hello, World!"};

private:
	byte msgdata[128]; // #TODO: see if this is enough?
	char cbuff[64];
};

#endif