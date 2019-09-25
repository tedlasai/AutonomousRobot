
#ifndef _SONAR_H_
#define _SONAR_H_

#include <Arduino.h>
#include "Display.h"
#include <NewPing.h>    //Ultrasonic sensor library

#define MAX_DISTANCE 200    //Maximum distance to ping (in cm). Maximum rated distance is 400-500cm.
#define TRIGGER_PIN  28   //Pin tied to trigger on the ultrasonic sensor.
#define ECHO_PIN     29   //Pin tied to echo on the ultrasonic sensor.

extern Display display;

class Sonar
{
public:
    Sonar();
    void init();
    int get();
    void print();
private:
    NewPing _sonar;
    int _dist;
};


#endif
