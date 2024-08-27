#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include <math.h>
#include <MsgPack.h>

#ifdef CONTROLTEST
extern void setup();
extern void loop();
#endif

#define PACKETDELIM "\n~"
#define LEN_SEP "~"

#define JOY_MAX 1024

struct ControlPacket
{
    bool a;
    bool b;
    int ljx = 125;
    int ljy = 126;
    MsgPack::str_t s = "Hello, Pi!";
    MSGPACK_DEFINE(a, b, ljx, ljy, s); // conversion to array -> [a, b, ljx, ljy, s]
};

void PrintPacket(ControlPacket *cmd);

// struct JoyPacket
// {
//     int jx;
//     int jy;
//     MSGPACK_DEFINE(jx,jy); // conversion to array -> [a, b, ljx, ljy,s]
// };

const float STEERANGLE_MAX_RAD = M_PI_4;
const u8_t STEERCTR_D_MIN = 200;  //#FIXME steer center dmin value
const float STEERCTR_SCALING = 1;

/* Steer Center DX/DY */
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


void CalcMotionVector(MotionVector *mvec, int joyx, int joyy, int throttle);

// MotionVector ProcessPacket(ControlPacket);

#endif