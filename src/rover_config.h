#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

#include <ArduinoUniqueID.h>

inline constexpr uint8_t DEVID_CONTROLLER = 0;
inline constexpr uint8_t DEVID_FL = 1;
inline constexpr uint8_t DEVID_FR = 2;
inline constexpr uint8_t DEVID_BL = 3;
inline constexpr uint8_t DEVID_BR = 4;
inline constexpr uint8_t DEVID_UNKNOWN = 5;

const byte UID_FL[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const byte UID_FR[8] = {0x56, 0x50, 0x50, 0x15, 0x20, 0x38, 0x34, 0x31};
const byte UID_BL[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const byte UID_BR[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// assumes the 8 byte UID
uint8_t get_driver_id();


inline constexpr uint8_t CMD_INTERVAL_MS = 20; // 50Hz - #LATER: what's the max?

#endif
