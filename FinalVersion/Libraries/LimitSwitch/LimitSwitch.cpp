
#include "LimitSwitch.h"

LimitSwitch::LimitSwitch(int left_pin, int right_pin):_left_pin(left_pin), _right_pin(right_pin){
}

void LimitSwitch::init(){
    pinMode(_left_pin, INPUT_PULLUP);
    pinMode(_right_pin, INPUT_PULLUP);
}

void LimitSwitch::get(){
    
    if (digitalRead(_left_pin) == HIGH  || digitalRead(_right_pin) == HIGH)
    {
        _hit = true;
    } else{
        _hit = false;
    }
}

void LimitSwitch::print(){
    if(_hit){
        display.print("Hit!", 0, 0);
    }else{
        display.clear();
    }
}


