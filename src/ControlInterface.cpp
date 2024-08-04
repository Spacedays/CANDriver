#ifndef CONTROL_INTERFACE
#define CONTROL_INTERFACE

#include <ControlInterface.h>
#include <Arduino.h>
// #include <MsgPacketizer.h>
// ------------------- Test 1: MsgPack Control Packet -------------------

#ifdef CONTROLTEST

MsgPack::Unpacker unpacker;
MsgPack::Packer packer;

ControlPacket cmd = ControlPacket();
char cbuff[64];
char c = '\0';
void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println(F("Setup complete."));
}

bool read_success;
byte msgdata[128]; // #TODO: see if this is enough?
u8_t i = 0;
uint msglen = 0;
long lasttime = 0;
void loop()
{
    // Read characters until '\n'. MsgPack messages will be preceeded by "\n~[msg_len]", then followed by the binary data
    while (c != '\n' && i < 64)
    {
        c = Serial.read();
        memcpy(&cbuff[i], &c, 1);
        i++;
    }
    if (i > 0)
    {
        // Received String - do something with it
        // ;   //#TODO
        i = 0;
    }
    else    // No message
    {
        return;
    }
    if (c == '\n')
    {
        c = Serial.read();
        if (c == '~')
        {
            // MessagePack message; Get Length of message and receive bytes
            msglen = Serial.parseInt();
            if (!msglen)
                return;
            Serial.readBytes(msgdata, msglen);
            if (!unpacker.feed(msgdata, msglen))
                return;
            if (unpacker.isArray()) //#TODO: define EXT array type for various control packets
                Serial.println(F("Unpacking array to object"));
                if (unpacker.deserialize(cmd))
                    PrintPacket(&cmd);
                else
                    Serial.println("Failed to deserialize");

            if (unpacker.isUInt8()) //#TODO: look for index before array to define message type? or find a way to do ext type
                i = unpacker.unpackUInt8();
            i=0;
        }
        else
        {
            // Message is a string; Reset and continue reading characters
            cbuff[i] = c;
            i++;
        }
    } else {
        //#TODO: Reached end of buffer while reading string; process part of string message
        Serial.print("Received partial string with length ");
        Serial.println(i);
        i=0;
    }
    if (micros() - lasttime > 5000)
    {
        Serial.print(F("Sending control packet:~"));
        packer.serialize(cmd);
        Serial.print(packer.size());
        Serial.print('~');
        Serial.write(* packer.data());
        Serial.println(F("~Finished writing control packet!"));
    }
    // bytes = Serial.available();
}

void PrintPacket(ControlPacket* cmd)
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