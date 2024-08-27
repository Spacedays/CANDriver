#ifndef CONTROL_INTERFACE
#define CONTROL_INTERFACE

#include "Arduino.h"
#include <math.h>
#include "ControlInterface.h"
// #include <MsgPacketizer.h>
// ------------------- Test 1: MsgPack Control Packet -------------------
// #define CONTROLTEST
// #define ACKPACKET
// #define ECHO_MSGPACK // #DEBUG - echo msgpack messages as string

void CalcSteerCenter(int16_t *d, int16_t *h, int joyx, int joyy)
{
    // sgn(joyx)*STEERCTR_D_MIN + STEERCTR_SCALING * atan2(JOY_MAX, joyx*M_PI_2);
    *d = sgn(joyx)*STEERCTR_D_MIN + STEERCTR_SCALING * tan(joyx*M_PI_2 / JOY_MAX + M_PI_2);
    // hmax = (abs(d) - STEERCTR_D_MIN)*tan(STEERANGLE_MAX_RAD)
    *h = joyy / JOY_MAX * (abs(*d) - STEERCTR_D_MIN)*tan(STEERANGLE_MAX_RAD);
}

void CalcMotionVector(MotionVector *mvec, int joyx, int joyy, int throttle)
{
    int16_t d, h;
    CalcSteerCenter(&d, &h, joyx, joyy);
    int SCdist[4] = {sqrt(sq(SCDY - h)+sq(-SCDX - d)), sqrt(sq(SCDY - h)+sq(SCDX - d)), sqrt(sq(-SCDY - h)+sq(-SCDX - d)), sqrt(sq(-SCDY - h)+sq(SCDX - d))};
    mvec->aFL = atan2(SCDY - h, -SCDX - d);
    mvec->aFR = atan2(SCDY - h, SCDX - d);
    mvec->aBL = atan2(-SCDY - h, -SCDX - d);
    mvec->aBR = atan2(-SCDY - h, SCDX - d);
    int m = max(max(SCdist[0], SCdist[1]), max(SCdist[2], SCdist[3]));

    mvec->vFL = SCdist[0]/m*throttle;
    mvec->vFR = SCdist[1]/m*throttle;
    mvec->vBL = SCdist[2]/m*throttle;
    mvec->vBR = SCdist[3]/m*throttle;
}


/* --- Test Code for controls --- */
#ifdef CONTROLTEST

MsgPack::Unpacker unpacker;
MsgPack::Packer packer;

ControlPacket cmd = {false, false, 125, 126, "Hello, World!"};
char cbuff[64];
char c = '\0';
void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println(F("Setup complete."));
}

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
        // Serial.print("-\n-Received Message: '");     // #DEBUG msg print
        // Serial.print(cbuff);
        // Serial.print("'");
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
                // Serial.printf("Rx Len: %d\n", msglen);   // #DEBUG rx length
                memset(lenbuf, 0, sizeof(lenbuf));

                if (!msglen)
                    return;
                j = Serial.readBytes(msgdata, msglen);
                if (j < msglen)
                {
                    Serial.printf("Incomplete Msgpack; waiting on %dB...", msglen - j);
                    Serial.readBytes(msgdata, msglen - j);
                }
                // Serial.print("Data: ");          //#DEBUG - data prints
                // Serial.write(msgdata, msglen);
                // Serial.printf("\nLen: %d\n", msglen);
                if (!unpacker.feed(msgdata, msglen))
                    return;
                // if (unpacker.isArray()) // #TODO: define EXT array type for various control packets
                // Serial.println(F("\r\nUnpacking array to object"));  //#DEBUG
                if (unpacker.deserialize(cmd))
                {
#ifdef ACKPACKET
                    Serial.println("ACK\r");
#endif
#ifdef ECHO_MSGPACK
                    PrintPacket(&cmd);
#endif
                    ;
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
            Serial.print(F("Received partial string with length "));
            Serial.println(i);
            i = 0;
        }
    }
    if (millis() - lastupdate > 5000)
    {
        // packer.to_array(cmd.a, cmd.b, cmd.ljx, cmd.ljy, cmd.s);
        packer.serialize(cmd);
        Serial.print(PACKETDELIM); // F("\n~"));
        Serial.print(packer.size());
        Serial.print(LEN_SEP); //'~');
            Serial.write((char *)packer.data(), packer.size());
        packer.clear();
        Serial.println(); // F("\nFinished writing control packet!"));
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

#endif

#endif