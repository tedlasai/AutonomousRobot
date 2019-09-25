
#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define DISPLAY_ADDRESS 0x27    //Maximum distance to ping (in cm). Maximum rated distance is 400-500cm.
#define DISPLAY_LENGTH 16   //Pin tied to trigger on the ultrasonic sensor.
#define DISPLAY_LINES  2   //Pin tied to echo on the ultrasonic sensor.

class Display
{
public:
    Display();
    void init();
    void print(String input);
    void print(String input, int line, int offset);
    void print(int input);
    void print(int input, int precision);
    void print(int input, int line, int offset);
    void print(double input);
    void print(double input, int precision);
    void print(double input, int line, int offset);
    void print(int32_t input);
    void print(int32_t input, int precision);
    void print(int32_t input, int line, int offset);
    void clear();
private:
    LiquidCrystal_I2C _lcd;
};


#endif
