
#ifndef _COMPASS_H_
#define _COMPASS_H_

#include <Arduino.h>
#include "HMC5883Llib.h"
#include <Adafruit_Sensor.h>
#include "Display.h"

#define X_MAX  78
#define X_MIN  -121
#define Y_MAX  60
#define Y_MIN  -142
#define COMPASS_OFFSET  -172
#define DECLINATION_ANGLE  0//0.1404990

const int X_MID = (X_MAX + X_MIN)/2;
const int Y_MID = (Y_MAX + Y_MIN)/2;

extern Display display;

class Compass
{
public:
    void init();
    void print();
    void printRaw();
    int get();
    void getRaw();
private:
    Magnetometer _mag;
    int x, y, z;
    int currentHeading;
};


#endif
