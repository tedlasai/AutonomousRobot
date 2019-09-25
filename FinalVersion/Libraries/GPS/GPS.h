
#ifndef _GPS_H_
#define _GPS_H_

#include <Arduino.h>
#include <TinyGPS.h>
#include "Display.h"

extern Display display;

class GPS
{
public:
    GPS();
    void init();
    void get();
    void print();
    int calculateTargetHeading();
    void printTargetHeading();
    float calculateDistance();
    void printDistance();
private:
    TinyGPS _gps; // create gps object
    float currentLat, currentLong;
    float targetLong, targetLat;
    int currentHeading, targetHeading;
    float targetDistance;
};


#endif
