
#include "Sonar.h"

extern Display display;

Sonar::Sonar(): _sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE){
}
void Sonar::init(){
    display.clear();
    display.print("Sonar Initializing",0,0);
    delay(1000);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

int Sonar::get(void)
{
    int avg_time = _sonar.ping_median(5);
    _dist = _sonar.convert_cm(avg_time);
    if (_dist == 0)
    {
        _dist = MAX_DISTANCE;
    }
    return _dist;
}
void Sonar::print(void)
{
    display.print("D:", 0, 10); // Prints string "Distance" on the LCD
    display.print(_dist); // Prints the distance value from the sensor
    if(_dist<10){
        display.print("  ");
    } else if(_dist<100){
        display.print(" ");
    }
    delay(10);
    
}
