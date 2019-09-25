/*
 * Reads the magnetic field in Gauss around the sensor
 */

#include <Wire.h>
#include "HMC5883Llib.h"


Magnetometer mag;
bool fail;

void setup()
{
   
}

void loop()
{
    // don't bother reading if we failed to connect
    if (fail)
        return;

    int x, y, z;

    int8_t ret = mag.readRaw(&x, &y, &z);
    switch (ret)
    {
        case HMC5833L_ERROR_GAINOVERFLOW:
            Serial.println("Gain Overflow");
            return;
        case 0:
            // success
            break;
        default:
            Serial.println("Failed to read Magnetometer");
            return;
    }
    

}
