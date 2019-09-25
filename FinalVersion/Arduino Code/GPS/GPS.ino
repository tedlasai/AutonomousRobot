#include "GPS.h"
#include "Display.h"

Display display;
GPS gps;

void setup(){ 
  display.init();
  gps.init();
} 

void loop(){ 
  gps.get();
  gps.print();
}


