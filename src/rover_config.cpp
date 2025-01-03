#include "rover_config.h"

// const uint8_t DEVID_CONTROLLER = 0;
// const uint8_t DEVID_FL = 1;
// const uint8_t DEVID_FR = 2;
// const uint8_t DEVID_BL = 3;
// const uint8_t DEVID_BR = 4;
// const uint8_t DEVID_UNKNOWN = 5;

uint8_t get_driver_id()
{
    if (memcmp(UniqueID8, UID_FL, sizeof(UID_FL)) == 0)
    {
        return DEVID_FL;
    }
    else if (memcmp(UniqueID8, UID_FR, sizeof(UID_FR)) == 0)
    {
        return DEVID_FR;
    }
    else if (memcmp(UniqueID8, UID_BL, sizeof(UID_FR)) == 0)
    {
        return DEVID_BL;
    }
    else if (memcmp(UniqueID8, UID_BR, sizeof(UID_FR)) == 0)
    {
        return DEVID_BR;
    }
    else
    {
        return DEVID_UNKNOWN;
    }
}