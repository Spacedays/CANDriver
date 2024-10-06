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

#define DEBUG

#ifdef DEBUG
#define D(x) x
#else
#define D(x)
#endif // Debug macro

void CalcSteerCenter(int16_t d, int16_t h, int joyx, int joyy)
{
	// sgn(joyx)*STEERCTR_D_MIN + STEERCTR_SCALING * atan2(JOY_MAX, joyx*M_PI_2);
	d = sgn(joyx) * STEERCTR_D_MIN + STEERCTR_SCALING * tan(joyx * M_PI_2 / JOY_MAX + M_PI_2);
	// hmax = (abs(d) - STEERCTR_D_MIN)*tan(STEERANGLE_MAX_RAD)
	h = joyy / JOY_MAX * (abs(d) - STEERCTR_D_MIN) * tan(STEERANGLE_MAX_RAD);
}

const float RAD2DEG = 180 / PI;
void CalcMotionVector(MotionVector *mvec, ControlPacket *cmd)
{
	int16_t d, h;
	CalcSteerCenter(d, h, cmd->ljx, cmd->ljy);
	CalcMotionVector(mvec, d, h, cmd->rt);
}

void CalcMotionVector(MotionVector *mvec, int16_t d, int16_t h, int throttle=1)
{
	int SCdist[4] = {sqrt(sq(SCDY - h) + sq(-SCDX - d)), sqrt(sq(SCDY - h) + sq(SCDX - d)), sqrt(sq(-SCDY - h) + sq(-SCDX - d)), sqrt(sq(-SCDY - h) + sq(SCDX - d))};
	mvec->aFL = atan2(SCDY - h, -SCDX - d) * RAD2DEG;
	mvec->aFR = atan2(SCDY - h, SCDX - d) * RAD2DEG;
	mvec->aBL = atan2(-SCDY - h, -SCDX - d) * RAD2DEG;
	mvec->aBR = atan2(-SCDY - h, SCDX - d) * RAD2DEG;
	int m = max(max(SCdist[0], SCdist[1]), max(SCdist[2], SCdist[3]));

	mvec->vFL = SCdist[0] / m * throttle;
	mvec->vFR = SCdist[1] / m * throttle;
	mvec->vBL = SCdist[2] / m * throttle;
	mvec->vBR = SCdist[3] / m * throttle;
}

/* --- MsgPackHandler --- */
PacketHandler::PacketHandler(ControlPacketCallback cmdcb, StrPacketCallback strcb)
{
	CmdCallback = cmdcb;
	StrCallback = strcb;
}

int PacketHandler::ParseSerial(const u8_t cbuff_start_idx)
{

	// -1: failed to parse
	// -2: message length header of 0
	// -3: unterminated string

	char c = '\0';

	bool partial;
	bool moredata;

	u8_t i = cbuff_start_idx;
	u8_t j = 0;
	char lenbuf[5]; // up to 99999 bytes?
	uint msglen = 0;

	// Read characters until '\n'. MsgPack messages will be preceeded by "\n~[msg_len]\n", then followed by the binary data
	while (Serial.available() && c != '\n' && i < 64)
	{
		c = Serial.read();
		memcpy(&cbuff[i], &c, 1);
		i++;
	}

	// Return if no data
	if (i == 0)
		return 0;

	// Received String - do something with it
	D(Serial.printf("-\n-Received Message w/ len %d: '%s'\n", i, cbuff);)
	if (c == '\n')
	{
		if (Serial.available() > 0)
		{
			if (Serial.peek() == LEN_SEP) // MsgPack message received; Process & handle message
			{
				c = Serial.read(); // consume LEN_SEP

				// Retrieve length of msgpack msg
				j = 0;
				c = Serial.read();
				while (c != LEN_SEP && j < 5)
				{
					lenbuf[j] = c;
					c = Serial.read();
					j++;
				}
				msglen = strtol(lenbuf, 0, 0);
				memset(lenbuf, 0, sizeof(lenbuf));

				if (!msglen)
					return -2;
				j = Serial.readBytes(msgdata, msglen);
				if (j < msglen)
				{
					Serial.printf("!Incomplete Msgpack; waiting on %dB...", msglen - j);
					Serial.readBytes(msgdata, msglen - j);
				}
				// #DEBUG - data prints
				D(Serial.print("Data: ");)
				D(Serial.write(msgdata, msglen);)
				D(Serial.printf("\nLen: %d\n", msglen);)
				if (!unpacker.feed(msgdata, msglen))
					return -1; // No data to unpack
				// if (unpacker.isArray()) // #TODO: define EXT array type or look at headers to handle various control packets
				if (unpacker.deserialize(cmd))
				{
#ifdef ACKPACKET
					Serial.println("ACK\r");
#endif
					CmdCallback(&cmd);
#ifdef ECHO_MSGPACK
					PrintPacket(&cmd);
#endif
					return 0;
				}
				unpacker.clear();
				Serial.println(F("\r\n!Failed to deserialize!\r"));
				return -2;
			}
			else // Received terminated string message, more data is still in rx queue
				partial = false;

			moredata = true;
		}
		partial = false; // Since no more RX characters, assume terminated string
		moredata = false;
	}
	else // Received unterminated string
	{
		// #TODO: Reached end of buffer while reading string; process part of string message
		Serial.print(F("\r\n!Received unterminated string with length "));
		Serial.println(i);
		if (i >= 63)
			moredata = true;
		else
			partial = true;
	}

	if (moredata) // recurse ParseSerial if more rx bytes to handle
	{
		if (i >= 63)
		{
			StrCallback(cbuff, i, false, true);
			D(Serial.println(F("\r\n!Recursing ParseSerial - buffer full"));)
			return ParseSerial();
		}
		else
		{
			D(Serial.println(F("\r\n!Recursing ParseSerial - More data available"));)
			return ParseSerial(i);
		}
	}
	StrCallback(cbuff, i, partial, false);
	return 0;
}

void PacketHandler::SendPacket(const ControlPacket *cmd)
{
	// packer.to_array(cmd.a, cmd.b, cmd.ljx, cmd.ljy, cmd.s);
	packer.serialize(*cmd);
	Serial.print(PACKETDELIM); // F("\n~"));
	Serial.print(packer.size());
	Serial.print(LEN_SEP); //'~');
	Serial.write((char *)packer.data(), packer.size());
	packer.clear();
	Serial.println(); // F("\nFinished writing control packet!"));
}

void PrintPacket(ControlPacket *cmd)
{
	ControlPacket cp = *cmd;
	// Serial.printf("CP: %d %d %d %d %s\n", cmd->a, cmd->b, cmd->rt, cmd->ljx, cmd->ljy, cmd->s);
	Serial.printf("CP: (%d,%d) %d (%d,%d) %s\n", cp.a, cp.b, cp.rt, cp.ljx, cp.ljy, cp.s);
};

/* --- Test Code for controls --- */

void PrintStrMsg(char *msg, uint strlen, bool partial, bool overflow)
{
	Serial.write(msg, strlen);
	Serial.println();
	if (partial)
		Serial.print("!Partial! ");
	if (overflow)
		Serial.print("!Overflow!");
}

#ifdef CONTROLTEST
#warning BUILDING CONTROLTEST


// void PrintSteerCenter()

PacketHandler pkt(PrintPacket, PrintStrMsg);

u16_t *steer_d;
u16_t *steer_h;

uint blinkduration = 300;
unsigned long lastupdate = 0;
long blinktime = 0;
bool blinkstate;

void setup()
{
	Serial.begin(115200);
	delay(2000);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	Serial.println(F("Setup complete."));
}

void loop()
{
	pkt.ParseSerial();

	if (millis() - lastupdate > 2000)	// #DEBUG Why does this break above 2000?
	{
		// packer.to_array(cmd.a, cmd.b, cmd.ljx, cmd.ljy, cmd.s);
		pkt.SendPacket(&(pkt.cmd));
		lastupdate = millis();
	}
	if (millis() - blinktime > blinkduration)
	{
		digitalWrite(LED_BUILTIN, blinkstate);
		blinkstate = !blinkstate;
		blinktime = millis();
	}
}

#endif

#endif