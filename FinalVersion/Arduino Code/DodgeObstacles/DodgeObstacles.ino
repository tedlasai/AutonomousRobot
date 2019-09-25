#include "Compass.h"
//#include "HMC5883Llib.h"
#include "Display.h"
#include "Sonar.h"
#include "Drive.h"
#include <Servo.h>
#include <GPSport.h>
#include <NMEAGPS.h>

Compass compass;
Display display;
Sonar sonar;
Drive drive;
static NMEAGPS gps;
gps_fix fix;    

int currentHeading;
int targetHeading;
float gpsDist;
int wayPoint = 0;
NeoGPS::Location_t wayPoints[] = {{ 389628803L, -1047539558}, { 389629935L, -1047541298}, { 389630855L, -1047539412}}; // WestLawn
//NeoGPS::Location_t wayPoints[] = {{ 388931945L, -1048021132}, { 388931945L, -1048021132}, { 388931945L, -1048021132}}; // WestLawn

static void GPSisr( uint8_t c )
{
  gps.handle( c );

} // GPSisr
void setup() {
  display.init();
  sonar.init();
  compass.init();
  drive.init();
  
  //initialize gps
  display.clear();
  bool gpsFound = false;
  gpsPort.begin(9600);
  gpsPort.attachInterrupt( GPSisr );
  DEBUG_PORT.begin( 9600 );
  
  while (gps.available(gpsPort) || !gpsFound) {
    fix = gps.read(); // save the latest
    display.print("GPS Initializing", 0, 0);
    if (fix.valid.location) {
        DEBUG_PORT.println("Valid");
        gpsFound = true;
    }
  }

  bool gpsValid = false;
  while (!gpsValid) {
    fix = gps.read(); // save the latest
    if (fix.valid.location) {
        display.clear();
        display.print("LAT:",0,0);
        display.print(long(fix.latitudeL()),10);
        display.print("LON:",1,0);
        display.print(long(fix.longitudeL()),10);
        delay(1000);
        display.clear();
        gpsValid = true;
        gpsDist = fix.location.DistanceKm( wayPoints[wayPoint] ) * 1000 ;
        targetHeading = fix.location.BearingToDegrees(wayPoints[wayPoint]);
    }
  }
}

void loop() {
    if (gps.available()) {
      fix = gps.read(); // save the latest
      if(wayPoint == 3){
        gpsDist = fix.location.DistanceKm( wayPoints[2] ) * 1000;
              targetHeading = fix.location.BearingToDegrees(wayPoints[2]);
         

      }else{
      gpsDist = fix.location.DistanceKm( wayPoints[wayPoint] ) * 1000;
            targetHeading = fix.location.BearingToDegrees(wayPoints[wayPoint]);
            if(gpsDist < SEARCH_START && wayPoint == 2){
        wayPoint++;
      }

      }
      if(gpsDist < ERROR_TOLERANCE){
        if(wayPoint !=3){
        wayPoint++;
        display.clear();
        display.print("Waypoint ", 0, 0);
        display.print(wayPoint, 1, 0);
        drive.incrementSpeed(3);
        if(wayPoint == 2){
        drive.incrementSpeed(6);
        }
        delay(1000);
        }
      }

      
      
      while (targetHeading < 0){
        targetHeading += 360;
      }
      while (targetHeading > 360){
        targetHeading -= 360;
      }
}
      if(wayPoint == 3){
        display.print("Searching");
          int scanAngle;
          if(gpsDist <= SEARCH_ERROR){
            //drive.stop(0);
           // scanAngle = sonar.scan();
            //drive.incrementSpeed(1);
          }
          //sonar.printAngle();
          currentHeading = compass.get();
          drive.findObject(0, targetHeading, currentHeading, gpsDist);
        
      } else{
      currentHeading = compass.get();
      int sonarDist = sonar.get();
      drive.turnByHeading(targetHeading, currentHeading, gpsDist);
      drive.moveAndAvoid(sonarDist, gpsDist); 

      
      display.print("T:",1,11); // Print a message to the LCD
      display.print(targetHeading); // Print a message to the LCD
      if(targetHeading<10){
        display.print("  ");
      } else if(targetHeading<100){
        display.print(" ");
      }
      
      display.print("H:",0,0); // Print a message to the LCD
      display.print(gpsDist,4); // Print a message to the LCD
      compass.print();
      sonar.print();
      }
      
}
