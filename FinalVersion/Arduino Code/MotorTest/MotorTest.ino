
#include "LimitSwitch.h"
#include "Drive.h"

Display display;

Drive drive;
void setup() {
      Serial.begin(115200); // connect serial 
      display.init();
      display.print("hi",0,0);
       drive.init();
       display.clear();
       
  // put your setup code here, to run once:
 //attach the steering and motor servos to pins 9 and 10 respectively


  
}

void loop() {
  /*drive.stop(5000);
  drive.right();
  drive.stop(5000);
  drive.straight();
  drive.stop(5000);
  drive.left();*/
  drive.left();
  drive.goForward(5000);
  drive.right();
  drive.backward(5000);
}



