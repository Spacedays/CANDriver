#ifndef CONTROL_INTERFACE
#define CONTROL_INTERFACE

#include <ControlInterface.h>
#include <Arduino.h>
// #include <MsgPacketizer.h>
// ------------------- Test 1: MsgPack Control Packet -------------------
// #define CONTROLTEST
// #define PRINTRXPACKET

#ifdef CONTROLTEST

MsgPack::Unpacker unpacker;
MsgPack::Packer packer;

ControlPacket cmd = { false, false, 125, 126, "Hello, World!"};

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
        Serial.print("-\nReceived String: ");
        Serial.println(cbuff);
        i = 0;
        if (c == '\n')
        {
            c = Serial.read();
            if (c == '~')
            {
                // MessagePack message; Get Length of message and receive bytes
                msglen = Serial.parseInt(); // terminates on reading a non-digit
                Serial.read();  // dispose of non-digit char
                if (!msglen)
                    return;
                Serial.readBytes(msgdata, msglen);
                if (!unpacker.feed(msgdata, msglen))
                    return;
                if (unpacker.isArray()) // #TODO: define EXT array type for various control packets
                    Serial.println(F("-\nUnpacking array to object"));
                if (unpacker.deserialize(cmd))
                    #ifdef PRINTRXPACKET
                    PrintPacket(&cmd);
                    #else
                    Serial.println("-\nReceived packet");
                    #endif
                else
                    Serial.println("-\nFailed to deserialize");

                if (unpacker.isUInt8()) // #TODO: look for index before array to define message type? or find a way to do ext type
                    i = unpacker.unpackUInt8();
                i = 0;
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
            Serial.print("Received partial string with length ");
            Serial.println(i);
            i = 0;
        }
    }
    if (millis() - lastupdate > 5000)
    {
        // packer.to_array(cmd.a, cmd.b, cmd.ljx, cmd.ljy, cmd.s);
        packer.serialize(cmd);
        Serial.print(F("Sending control packet:\n~"));
        Serial.print(packer.size());
        Serial.print('~');
        Serial.write((char*)packer.data());
        packer.clear();
        Serial.println(F("-\nFinished writing control packet!"));
        lastupdate = millis();
    }
    if (millis() - blinktime > 200)
    {
        digitalWrite(LED_BUILTIN, blinkstate);
        blinkstate = !blinkstate;
        blinktime = millis();
    }
}

void PrintPacket(ControlPacket *cmd)
{
    Serial.print(cmd->a);
    Serial.print(" ");
    Serial.println(cmd->b);
    Serial.print(cmd->ljx);
    Serial.print(" ");
    Serial.println(cmd->ljy);
    Serial.println(cmd->s);
}

// MsgPack::map_t<String, int> mp {{"a", 1}, {"b", 2}, {"c", 3}};  // json {{"a", 1}, {"b", 2}, {"c", 3}}

// const uint8_t RECV_INDEX = 0x21;
// const uint8_t SEND_INDEX = 0x22;

// void setup()
// {
//     Serial.begin(115200);
//     delay(2000);
//     Serial.print("Setup complete.");

//     // update received data directly - taken from map_bind_to_variables.ino
//     MsgPacketizer::subscribe(Serial, RECV_INDEX, cmd);

//     // register variables to publish repeatedly
//     MsgPacketizer::publish(Serial, SEND_INDEX, cmd)
//         ->setFrameRate(10.f);
// }

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