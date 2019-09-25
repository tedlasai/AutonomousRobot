/*
* Ultrasonic Sensor HC-SR04 and Arduino Tutorial
*
* Crated by Dejan Nedelkovski,
* www.HowToMechatronics.com
*
*/

#include "Display.h"
#include "Sonar.h"
#include <Servo.h>


Display display;
Sonar sonar;

int dist;

void setup() {
Serial.begin(115200);
sonar.init();  
display.init();
//sonarServo.attach(4);

}
void loop() {
int x= sonar.scan();
sonar.printAngle();
}

