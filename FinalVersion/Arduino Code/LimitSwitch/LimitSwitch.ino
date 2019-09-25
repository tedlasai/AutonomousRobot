// include the library code
#include <LimitSwitch.h> 
#include <LiquidCrystal_I2C.h>

Display display;
LimitSwitch limitSwitch(2,3) ;

void setup() {
  display.init();
  limitSwitch.init();
}

void loop() {
  limitSwitch.get();
  limitSwitch.print();
}



