
#ifndef _LIMITSWITCH_H_
#define _LIMITSWITCH_H_

#include <Arduino.h>
#include "Display.h"

extern Display display;
class LimitSwitch
{
public:
    LimitSwitch(int left_pin, int right_pin);
    void init();
    void get();
    void print();
private:
    int _left_pin, _right_pin;
    int _hit;
};


#endif
