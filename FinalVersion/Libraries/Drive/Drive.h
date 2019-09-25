
#ifndef _DRIVE_H_
#define _DRIVE_H_

#include <Servo.h>    //Servo library, included with the Arduino IDE
#include <Arduino.h>
#include "Display.h"
#include "LimitSwitch.h"

extern Display display;

#define GO 1640     //PPM speed for forward motion on motor servo, 1665-1695 in long grass (like
//the west lawn) 1600 is preferable for solid ground (cement, gravel, etc) 
#define BACK 1100      //PPM speed for reverse motion on motor servo,
//1000 in long grass, 1200 for solid grounds
#define STOP 1500     //PPM speed for idle on the motor servo
#define STRAIGHT 90   //Steering servo straight command
#define LEFT 140      //Steering servo left turn command
#define RIGHT 50      //Steering servo right turn command
#define SMALL_LEFT 100      //Steering servo left turn command
#define SMALL_RIGHT 80      //Steering servo right turn command
#define BACK_DELAY 50 //delay from switching from forward to back
#define DELAY_TIME 1 //delay period for each delay call
#define LEFT_PIN  2
#define RIGHT_PIN 3
#define MOTOR_PIN 10
#define STEER_PIN 9
#define ERROR_HEADING 5   //error, in degrees, to ignore between current heading and target heading
#define ERROR_TOLERANCE 1.5 //distance between target waypoint and current waypoint in which to stop and consider it good, in meters
#define SMALL_TURN_TOLERANCE 10 //for which angles to turn slightly
#define LARGE_TURN_DISTANCE 15 //distance where it is ok to use larger turns

#define UPHILL_HEADING 60   //Heading that results in largest slope, making motor work harder
#define GO_ADJUST 30    //Value to add/subtract to/from GO based on heading





class Drive
{
public:
    Drive ();
    void left();
    void right();
    void straight();
    void smallLeft();
    void smallRight();
    void init();
    void forward(int delay);
    void goForward(int delay);
    void backward(int delay);
    void stop(int delay);
    void driveDelay(int delay);
    void turnByHeading(int targetHeading, int currentHeading, float gpsDistance);
    void moveAndAvoid (int distance, float gpsDistance);
    void findObject(int sonarDistance);
    static void pressed();
    static void endlessLoop();
    static bool switchPressed;
private:
    LimitSwitch _limitSwitch;
    int direction;
    float slope;
    float go_north;
    int go_forward;
};

#endif
