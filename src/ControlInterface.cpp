#ifndef CONTROL_INTERFACE
#define CONTROL_INTERFACE

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

#pragma region MotionVectorOperators
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
//  Full multiplication
MotionVector operator*(const MotionVector &lhs, const float mult)
{
	MotionVector mvec = {
		int(float(lhs.aFL) * mult),
		int(float(lhs.aFR) * mult),
		int(float(lhs.aBL) * mult),
		int(float(lhs.aBR) * mult),
		int(float(lhs.vFL) * mult),
		int(float(lhs.vFR) * mult),
		int(float(lhs.vBL) * mult),
		int(float(lhs.vBR) * mult),
	};
	return mvec;
}
// Angle multiplication
MotionVector operator%(const MotionVector &lhs, const float mult)
{
	MotionVector mvec = {
		int(float(lhs.aFL) * mult),
		int(float(lhs.aFR) * mult),
		int(float(lhs.aBL) * mult),
		int(float(lhs.aBR) * mult),
		lhs.vFL,
		lhs.vFR,
		lhs.vBL,
		lhs.vBR,
	};
	return mvec;
}
#pragma endregion MotionVectorOperators

void CalcSteerCenter(float *d, float *h, int joyx, int joyy)
{
	// jx scales the angle of the front right wheel while assuming h=0
	// jy is multipied by the max height (limited by steering angle) at the given distance

	float jx = float(joyx) / float(JOY_MAX);
	float jy = float(joyy) / float(JOY_MAX);
	if (abs(jx * STEERANGLE_MAX_RAD) < STEERANGLE_MIN_RAD)
	{
		*d = 0;
		*h = 0;
		return;
	}
	*d = sgn(joyx) * SCDX + SCDY / tan(jx * STEERANGLE_MAX_RAD);
	// d_min = SCDX + SCDY * tan(STEERANGLE_MAX_RAD)
	// h_max = (abs(d)-d_min)  / tan(STEERANGLE_MAX_RAD)
	*h = -jy * (abs(*d) - (SCDX + SCDY * tan(STEERANGLE_MAX_RAD))) * tan(STEERANGLE_MAX_RAD);
}

const float RAD2DEG = 180 / PI;
void CalcMotionVector(MotionVector *mvec, ControlPacket *cmd)
{
	float d, h;
	CalcSteerCenter(&d, &h, cmd->ljx, cmd->ljy);
	CalcMotionVector(mvec, d, h, cmd->rt);
}

//
void CalcMotionVector(MotionVector *mvec, float d, float h, int16_t throttle = 0)
{
	float SCdist[4] = {sqrt(sq(d + SCDX) + sq(h - SCDY)),
					   sqrt(sq(d - SCDX) + sq(h - SCDY)),
					   sqrt(sq(d + SCDX) + sq(h + SCDY)),
					   sqrt(sq(d - SCDX) + sq(h + SCDY))};
	mvec->aFL = int(atan((h - SCDY) / (d + SCDX)) * RAD2DEG);
	mvec->aFR = int(atan((h - SCDY) / (d - SCDX)) * RAD2DEG);
	mvec->aBL = int(atan((h + SCDY) / (d + SCDX)) * RAD2DEG);
	mvec->aBR = int(atan((h + SCDY) / (d - SCDX)) * RAD2DEG);
	float m = max(max(SCdist[0], SCdist[1]), max(SCdist[2], SCdist[3]));

	mvec->vFL = int(SCdist[0] / m * float(throttle));
	mvec->vFR = int(SCdist[1] / m * float(throttle));
	mvec->vBL = int(SCdist[2] / m * float(throttle));
	mvec->vBR = int(SCdist[3] / m * float(throttle));
}

float ThrottleToFloat(int throttleval)
{
	return float(throttleval) * RT_MAX;
}

#pragma region MsgPackHandler
/* --- MsgPackHandler --- */
PacketHandler::PacketHandler(ControlPacketCallback cmdcb, StrPacketCallback strcb)
{
	CmdCallback = cmdcb;
	StrCallback = strcb;
}

/// @brief Parses serial input for a msgpack message, potentially triggering CmdCallback and/or StrCallback
/// @param cbuff_start_idx
/// @return 2: rx string 1: successful cmd 0: no data -1: Msgpack length is 0; -2: no or malformed data after header; -3: failed deserialization
int PacketHandler::ParseSerial(const uint8_t cbuff_start_idx)
{
	// 2: string
	// 1: control packet
	// 0: no data
	// -1: message length header of 0
	// -2: no data to unpack after header or data is malformed
	// -3: data could not be deserialized

	char c = '\0';

	bool partial;
	bool moredata;

	uint8_t i = cbuff_start_idx;
	uint8_t j = 0;
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
					return -1;
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
					return -2; // No data to unpack after header or data is malformed
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
					return 1;
				}
				unpacker.clear();
				Serial.println(F("\r\n!Failed to deserialize!\r"));
				return -3; // data could not be deserialized
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
	return 2;
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

#pragma endregion MsgPackHandler

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

PacketHandler handler(PrintPacket, PrintStrMsg);

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
	handler.ParseSerial();

	if (millis() - lastupdate > 2000) // #DEBUG Why does this break above 2000?
	{
		// packer.to_array(cmd.a, cmd.b, cmd.ljx, cmd.ljy, cmd.s);
		handler.SendPacket(&(handler.cmd));
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