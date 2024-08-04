#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include <MsgPack.h>

#ifdef CONTROL_TEST
extern void setup();
extern void loop();
#endif

struct ControlPacket
{
    bool a;
    bool b;
    int ljx;
    int ljy;
    MsgPack::str_t s;
    MSGPACK_DEFINE(a,b,ljx,ljy,s); // conversion to array -> [a, b, ljx, ljy, s] 
};

void PrintPacket(ControlPacket* cmd);

// struct JoyPacket
// {
//     int jx;
//     int jy;
//     MSGPACK_DEFINE(jx,jy); // conversion to array -> [a, b, ljx, ljy,s] 
// };

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
    MSGPACK_DEFINE(vFL,vFR,vBL,vBR,aFL,aFR,aBL,aBR);
};

// MotionVector ProcessPacket(ControlPacket);

 #endif