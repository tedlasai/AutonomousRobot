
#include "Display.h"
#include "Sonar.h"
#include "Drive.h"

Display display;
Sonar sonar;
Drive drive;

void setup() {
  Serial.begin(115200); 
  display.init();
  drive.init();
  sonar.init();
  display.clear();
}

void loop() {
  int sonarDist = sonar.get();
  sonar.print();
  drive.moveAndAvoid(sonarDist);
}
