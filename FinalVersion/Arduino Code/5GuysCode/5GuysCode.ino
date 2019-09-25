//This is the final code for the Tekampbot project used in Fall 2017 by the 
//Five Guys and a Bot team. 


//Library inclusions
#include <Wire.h>      //arduino wiring library, included with the Arduino IDE
#include <Adafruit_Sensor.h>  //driver library for LSM303, available at https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_LSM303_U.h>  //magnetometer library, available at
//https://github.com/adafruit/Adafruit_LSM303DLHC
#include <Adafruit_GPS.h>   //GPS library, available at
//https://github.com/adafruit/Adafruit_GPS
#include <math.h>     //used for GPS calculations (distanceTo and courseTo), basic C header file
#include <Servo.h>    //Servo library, included with the Arduino IDE
#include <NewPing.h>    //Ultrasonic sensor library, available at https://bitbucket.org/teckel12/arduino-new-ping/downloads/
#include <LiquidCrystal_I2C.h>  //LCD library, available at https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library

//Preprocessor definitions
#define GPSSerial Serial1   //defining the Hardware Serial port being used
#define ERROR_TOLERANCE 0.1 //distance between target waypoint and current waypoint in which to stop and consider it good, in meters
#define ERROR_HEADING 5   //error, in degrees, to ignore between current heading and target heading
#define GO 1665     //PPM speed for forward motion on motor servo, 1665-1695 in long grass (like
//the west lawn) 1600 is preferable for solid ground (cement, gravel, etc) 
#define BACK 980      //PPM speed for reverse motion on motor servo,
//1000 in long grass, 1200 for solid ground
#define STOP 1500     //PPM speed for idle on the motor servo
#define STRAIGHT 90   //Steering servo straight command
#define LEFT 120      //Steering servo left turn command
#define RIGHT 60      //Steering servo right turn command
#define TRIGGER_PIN  13   //Pin tied to trigger on the ultrasonic sensor.
#define ECHO_PIN     13   //Pin tied to echo on the ultrasonic sensor.
#define MAX_DISTANCE 200    //Maximum distance to ping (in cm). Maximum rated distance is 400-500cm.

#define UPHILL_HEADING 30   //Heading that results in largest slope, making motor work harder
#define GO_ADJUST 10    //Value to add/subtract to/from GO based on heading

//Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial terminal console
//Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

//instantiation of magnetometer object and assign it a unique ID of 12345
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//instantiation of GPS object and connection to the serial line on the hardware port used
Adafruit_GPS GPS(&GPSSerial);
     
//create servo objects
Servo steer;
Servo motor;

// LCD Display, use I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);  //Set the LCD I2C address and size (4x20)

//US sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
//unsigned int for setting up serial terminal debugging
uint32_t timer = millis();

//double array to hold target waypoints, set of waypoints used is for west lawn
float WPlist[2][4]={{38.893188,38.893115,38.893093,0},{-104.802291,-104.802444,-104.802345,0}};


//global variables to aid in navigations
float currentLat,
      currentLong,
      targetLat,
      targetLong;
int targetHeading;              
int currentHeading;
int headingError; 
float distanceToTarget;
int currWP = 1;
//US sensor nose distance
int dist;
int touch = 0;

float slope = 2*GO_ADJUST/(360-UPHILL_HEADING);
float go_north = GO + GO_ADJUST;
int go_forward = GO;

//setup portion of program, used to initialize components and serial lines
void setup(void)
{

  //init LCD
  lcd.begin();
  //lcd.backlight(); //uncomment if running indoors
  lcd.clear();
  lcd.print("Tekampbot Startup");
  
  //setup interrupt for touch sensor
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), touchthis, RISING);

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  //check for compass
  if(!mag.begin())
  { 
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  //Connect serial terminal at 115200 so we can read the GPS fast enough and echo without dropping chars for debugging
  Serial.begin(115200);
  
  Serial.println("Tekampbot startup protocols!");
  
  //start hardware serial communications
  //9600 baud rate is the default baud rate for Adafruit MTK GPS, and the Neo 6M module used in our prototypes (some use 4800)
  GPS.begin(9600); 
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  //This line will setup the GPS to turn on only the "minimum recommended" data (RMC) sentence
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  //To ensure that the parsing code works well and has time to sort the data, as well as
  //print it out, it is not suggested to use anything higher than 1 Hz
  //Set the GPS update rate, the Neo 6M has a max refresh of 5Hz while the adafruit has a max of 10Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);

  //uncomment next line to get antenna update status 
  //GPS.sendCommand(PGCMD_ANTENNA);

  //wait for fix, updating display with each new NMEA sentence received
  unsigned long startTime = millis();
  while (!GPS.fix)                      
    {
      processGPS();
      Serial.print(F("Wait Time: "));
      Serial.println((int) (millis() - startTime) / 1000); 
      if (GPS.newNMEAreceived())
        GPS.parse(GPS.lastNMEA());      
    }
      
  //set the initial target waypoint
  targetLat = WPlist[0][0];
  targetLong = WPlist[1][0];
  
  //give one second for all serial comms to level out and be ready
  delay(1000); 
  
  //attach the steering and motor servos to pins 9 and 10 respectively
  steer.attach(9);
  motor.attach(10);

  //initialize servos to straight position and idle speed
  steer.write(STRAIGHT);
  delay(1000);
  motor.write(STOP);
  delay(500);
  
}

void loop(void)
{
  //GPS count variable to ensure we get a full sentence on each call of the processGPS function
  unsigned long GPScount = millis(); 

  //loop through GPS parsing for half a second to ensure updated positioning
  while(millis()-GPScount < 500)
  {
  processGPS();
  }
  
  //get latitude and longitude in format that we can use
  normalizeGPS();
  
  //uncomment next four lines for debugging commands to print to the Serial monitor
  lcd.clear();
  lcd.print("lat: "); lcd.print(currentLat, 6);
  lcd.setCursor(0,1); lcd.print("lng: "); lcd.print(currentLong, 6);
  //Serial.print("Location: ");
  //Serial.print(currentLat, 6); //Serial.print(GPS.lat);
  //Serial.print(", ");
  //Serial.println(currentLong, 6); //Serial.println(GPS.lon);

  //calculate distance between target waypoint and current waypoint
  distanceToTarget = distanceToWaypoint();
  
  //more debugging commands
  //Serial.println(distanceToTarget);

  //calculate heading to target waypoint in integer degrees
  courseToWaypoint();

  //read and average compass heading for current heading in integer degrees  
  currentHeading = avgCompass();

  //Adjust motor power using heading to help going uphill
  if(currentHeading < UPHILL_HEADING){
    go_forward = (int)(go_north + slope*(currentHeading - UPHILL_HEADING));
  }
  if(currentHeading >= UPHILL_HEADING){
    go_forward = (int)(go_north - slope*(currentHeading - UPHILL_HEADING));
  }

  //more debugging
  //Serial.print("TargHeading: ");Serial.print(targetHeading);Serial.write(0xC2); Serial.write(0xB0);Serial.println(" ");
  //Serial.print("Heading: ");Serial.print(currentHeading);Serial.write(0xC2); Serial.write(0xB0);Serial.println(" ");

  //calculate optimal turn based on GPS readings without Obstacle avoidance
  CalculateTurnByHeading(targetHeading, currentHeading);

  //check ultrasonic sensor for obstacle distance
  CheckUS();

  //using nose distance as passed value determine movement
  MoveAndAvoid(distanceToTarget);

  //more debugging commands
  //Serial.print(targetLat, 6);Serial.print(", ");Serial.println(targetLong, 6);
  Serial.print("nose dist: "); Serial.print(dist); Serial.println("cm");
  //Serial.print("Touch value: "); Serial.println(touch);

  if (targetLat == 0 && targetLong == 0)
  {
    motor.write(STOP);
    //endlessLoop();
    CheckUS();
    while(dist >= MAX_DISTANCE)
    {
      
      steer.write(LEFT);
      motor.write(go_forward);
      delay(1000);
      motor.write(STOP);
      CheckUS();
    }
    while(dist < MAX_DISTANCE)
    {
      steer.write(STRAIGHT);
      motor.write(go_forward);
    }  
  }
  
}


//conversion for lat/long into decimal degrees from degrees in minutes
double convertDegMin (float degMin) 
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}

//determine course heading 
int courseToWaypoint() 
{
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  targetHeading = degrees(a2);
  return targetHeading;
}

//distance calculation done using the Haversine formula of the Law of Cosines
//for great sphere mathematics, but since the earth is 
//not a perfect sphere there will be about a 0.5% error in the formula
float distanceToWaypoint() 
  float R = 6371e3; // metres
  float phi1 = lat1.toRadians();
  float phi2 = lat2.toRadians();
  float deltaPhi = (lat2-lat1).toRadians();
  float deltaLambda = (lon2-lon1).toRadians();

  float a = Math.sin(deltaPhi/2) * Math.sin(deltaPhi/2) +
        Math.cos(phi1) * Math.cos(phi2) *
        Math.sin(deltaLamda/2) * Math.sin(deltaLambda/2);
  float c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

  float d = R * c;
  
  return d;
}  

//compass averaging function
int avgCompass(void)
{
  //average of 5 readings of the compass to help with accuracy
  int holder=0;
  for (int i=0; i <= 4; i++){
      holder += readCompass();
   }
  return (holder/5);
}

//function to read compass values and convert them into human readable degrees
int readCompass(void)
{
  //create a sensor event and pass by reference to the magnetometer
  sensors_event_t event;
  mag.getEvent(&event);

  //magnetometer reads in x,y,z coords and uT (microteslas)
  //normalize to radians in the x,y plane 
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  //include the Declination angle in radians that occurs in Colorado Springs
  #define DEC_ANGLE 0.1404990
  heading += DEC_ANGLE;
  
  //Correct heading for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  //Check for wrap due to addition of declination angle
  if(heading > 2*PI)
    heading -= 2*PI;
   
  //Convert from radians to degrees
  float headingDegrees = heading * 180/M_PI; 

  //return the heading in integers for ease of use
  return ((int)headingDegrees); 
}

//function to retrieve characters from the NMEA sentence that the GPS module
//has currently stored and check if it is an updated sentence. Debugging 
//commands are available to print to the serial terminal if needed
void processGPS(void)
{
  //read data from the GPS inside the processGPS loop in the 'main loop'
  char c = GPS.read();
  
  //allows debugging if the GPSECHO definition is set to true
  if (GPSECHO)
    if (c) Serial.print(c);
  
  //When a sentence is received we check the checksum, and parse it
  if (GPS.newNMEAreceived()) 
  {
    //The thing here is if we print data
    //we may end up not listening and catching other sentences,
    //this is why we set for RMC only, or at worse case scenario RMC and GGA sentences
    
    Serial.println(GPS.lastNMEA()); //this will set the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; //Though we can fail in parsing a sentence, therefore just wait for another
  }
  
  //if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();

//uncomment these lines to see debugging information on the serial terminal.     
//  //Every 2 seconds, print parsed data for debugging and verifying all is working as intended
//  if (millis() - timer > 2000) 
//  {
//    timer = millis(); // reset the timer
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
//    if (GPS.fix) 
//    {
//      Serial.print("Location: ");
//      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
//      Serial.print(", ");
//      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
//    }
//  }
}

//Function to process GPS lat/long into signed decimal degrees for calculations
void normalizeGPS(void)
{
    currentLat = convertDegMin(GPS.latitude);
    currentLong = convertDegMin(GPS.longitude);

  //make the Long/Lat signed
    if (GPS.lat == 'S')
    {
      currentLat = -currentLat;
    }
    if (GPS.lon = 'W')
    {  
      currentLong = -currentLong; 
    }
}

//loop to stop all functionality at the end
void endlessLoop(void)
{
  while (1)
  {
    motor.write(STOP);
    //Serial.println("done!");
    lcd.clear();
    lcd.print("DONE!");
    delay(1500);
  }
}

//function to calculate turns based solely off the GPS without regard to Obstacles
void CalculateTurnByHeading(int targetHeading, int currentHeading)
{
  headingError = (targetHeading - currentHeading);

  //put headingerror into turning degrees
  if (headingError < -180)
    headingError +=360;
  if (headingError > 180)
    headingError -=360;

  if (abs(headingError) <= ERROR_HEADING)
    {
      steer.write(STRAIGHT);
    }
  else if (headingError < 0)
    {
      steer.write(LEFT);
    }
  else if (headingError > 0)
    {
      steer.write(RIGHT);
    }
  else
    {
      steer.write(STRAIGHT);
    }
}

//function to initiate movement and account for obstacles
void MoveAndAvoid (int distance)
{
  if (distance <= ERROR_TOLERANCE)
  {
    motor.write(STOP);
    currWP++;
    delay(5000);
    GetWaypoint(currWP);
  }
  //comment all else-ifs and uncomment else statement to run solely off GPS navigations without regard to Obstacle avoidance
//  else
//  {
//    motor.write(go_forward);
//  }
  else if (dist >= 190)
  {
    motor.write(go_forward);
  }
  else if (dist < 190 && dist >= 100)
  {
    motor.write(STOP);
    delay(250);
    steer.write(LEFT);
    motor.write(go_forward);
    delay(3000);
  }
  else if (dist < 100)
  {
    motor.write(STOP);
    delay(1000);
    steer.write(LEFT);
    delay(250);
    motor.write(STOP);
    delay(2000);
    motor.write(BACK);
    delay(2000);
    motor.write(STOP);
    delay(2000);
    motor.write(BACK);
    delay(1000);
    motor.write(STOP);
    delay(3000);
    steer.write(STRAIGHT);
    motor.write(go_forward);
    delay(2000);
  }
}

//function to check ultrasonic sensor, using a median of 5 readings while
//tossing out the out of range readings
void CheckUS(void)
{
  int avg_time = sonar.ping_median(5);
  dist = sonar.convert_cm(avg_time);
  if (dist == 0)
  {
    dist = MAX_DISTANCE;
  }
}

//function to update target waypoints via switch statement
void GetWaypoint(int currWP)
{
  switch (currWP)
  {
    case 1:
       targetLat = WPlist[0][0];
       targetLong = WPlist[1][0];
       break;
    case 2:
       targetLat = WPlist[0][1];
       targetLong = WPlist[1][1];
       break;
    case 3:
       targetLat = WPlist[0][2];
       targetLong = WPlist[1][2];
       break;
    case 4:
       targetLat = WPlist[0][3];
       targetLong = WPlist[1][3];
  }
}
//function for touch sensor interrupt service routine
void touchthis()
{
  endlessLoop();
  //touch = 1;
}

