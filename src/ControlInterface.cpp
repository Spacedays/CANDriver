#ifndef CONTROL_INTERFACE
#define CONTROL_INTERFACE

#include <ControlInterface.h>
#include <Arduino.h>
// #include <MsgPack.h>
#include <MsgPacketizer.h>
// ------------------- Test 1: MsgPack Control Packet -------------------

#define CONTROLTEST

#ifdef CONTROLTEST

uint8_t sbuff[64];
unsigned char cbuff[64];
// MsgPack::Unpacker unpacker;
// MsgPack::Packer<MsgPack::sbuffer> pk(&sbuf);

ControlPacket cmd = ControlPacket();

// MsgPack::map_t<String, int> mp {{"a", 1}, {"b", 2}, {"c", 3}};  // json {{"a", 1}, {"b", 2}, {"c", 3}}

const uint8_t RECV_INDEX = 0x21;
const uint8_t SEND_INDEX = 0x22;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.print("Setup complete.");

    // update received data directly - taken from map_bind_to_variables.ino
    MsgPacketizer::subscribe(Serial, RECV_INDEX, cmd);

    // register variables to publish repeatedly
    MsgPacketizer::publish(Serial, SEND_INDEX, cmd)
        ->setFrameRate(10.f);
}

int bytes;
void loop()
{
//     bytes = Serial.available();
//     if (bytes)
//     {
//         Serial.readBytes(sbuff, bytes);
//         unpacker.feed(cbuff, bytes);
//         if (unpacker.unpackable(&cmd))
//         {
//             unpacker.unpack(cmd);
//         }
//     }
    MsgPacketizer::update();
}

/*
void setup()
{
    Serial.begin(115200);
    Serial.print("Setup complete.");
    unpacker = MsgPack::Unpacker();

}

int bytes;
void loop()
{
    bytes = Serial.available();
    if(bytes){
        Serial.readBytes(sbuff, bytes);
        unpacker.feed(cbuff, bytes);
        if (unpacker.unpackable(&cmd)) {
            unpacker.unpack(cmd);
        }
    }
}
*/

#endif

#endif