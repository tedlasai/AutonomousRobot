
#include "Drive.h"


Servo steer;
Servo motor;

bool Drive::switchPressed = false;

Drive::Drive():_limitSwitch(LEFT_PIN, RIGHT_PIN){
}
void Drive::init(){
    display.clear();
    display.print("Drive Initializing",1,0);
    steer.attach(STEER_PIN);
    motor.attach(MOTOR_PIN);
    _limitSwitch.init();
    attachInterrupt(digitalPinToInterrupt(LEFT_PIN), pressed, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), pressed, FALLING);
    slope = 2*GO_ADJUST/(360-UPHILL_HEADING);
    go_north = GO + GO_ADJUST;
    go_forward = GO;
    straight();
    stop(5000);
}

void Drive::backward(int delay){
        //display.print("Bac",0,0);
        motor.write(BACK);
        driveDelay(BACK_DELAY);
        motor.write(STOP);
        driveDelay(BACK_DELAY);
        motor.write(BACK);
        driveDelay(delay);
}

void Drive::goForward(int delay){
        //display.print("For",0,0);
        motor.write(go_forward);
        driveDelay(delay);
}

void Drive::left(){
    direction = 0;
    //display.print("Lef",0,5);
    steer.write(LEFT);
}
void Drive::straight(){
    direction = 1;
    //display.print("Str",0,5);
    steer.write(STRAIGHT);
}
void Drive::smallRight(){
    direction = 2;
    //display.print("SRi",0,5);
    steer.write(SMALL_RIGHT);
}

void Drive::smallLeft(){
    direction = 0;
    //display.print("SLe",0,5);
    steer.write(SMALL_LEFT);
}


void Drive::right(){
    direction = 2;
    //display.print("Rig",0,5);
    steer.write(RIGHT);
}
void Drive::stop(int delay){
    //display.print("Sto",0,0);
    motor.write(STOP);
    driveDelay(delay);
}

void Drive::turnByHeading(int targetHeading, int currentHeading, float gpsDistance)
{
    float headingError = ((targetHeading - currentHeading)+1080)%360;
    
    if (headingError < -180)
        headingError +=360;
    if (headingError > 180)
        headingError -=360;
    
    //Adjust motor power using heading to help going uphill
    if(currentHeading < UPHILL_HEADING){
        go_forward = (int)(go_north + slope*(currentHeading - UPHILL_HEADING));
    }
    if(currentHeading >= UPHILL_HEADING){
        go_forward = (int)(go_north - slope*(currentHeading - UPHILL_HEADING));
    }
    
    //Serial.print("Direction: ");
    //Serial.println(headingError);
    /*if (abs(headingError) <= ERROR_HEADING)
    {
        straight();
    }
    else if (headingError < 0)
    {
        left();
    }
    else if (headingError > 0)
    {
        right();
    }
    else
    {
        straight();
    }
*/
     if (abs(headingError) <= ERROR_HEADING)
     {
     straight();
     }
     else if (headingError > 0)
     {
         if(headingError > SMALL_TURN_TOLERANCE || gpsDistance > LARGE_TURN_DISTANCE)
            right();
         else
            smallRight();
     }
     else if (headingError < 0)
     {
         if(headingError < -SMALL_TURN_TOLERANCE || gpsDistance > LARGE_TURN_DISTANCE)
             left();
         else
             smallLeft();
     }
     else
     {
     straight();
     }
}

void Drive::moveAndAvoid (int sonarDistance, float gpsDistance)
{
    if (gpsDistance <= ERROR_TOLERANCE)
    {
        endlessLoop();
    }
    else if (sonarDistance >= 60)
    {
        goForward(0);
        //Serial.println("greater than 190");
    }
    else if (sonarDistance < 60 && sonarDistance >= 50)
     {
         if(direction == 2){
             right();
         }else{
             left();
         }
         goForward(200);
         //Serial.println("greater than 100");
     }
     else if (sonarDistance < 50)
     {
         stop(1000);
         if(direction == 0){
             right();
         } else{
             left();
         }
         stop(2000);
         backward(3000);
         stop(3000);
         //Serial.println("less than 100");
         
     }
}



void Drive::driveDelay(int delay){
    unsigned long startMillis = millis();
    unsigned long currentMillis = millis();
    while((currentMillis - startMillis < delay) && !switchPressed) {
        currentMillis = millis();
        // save the last time you blinked the LED
    }
    if(switchPressed){
        endlessLoop();
    }
}

void Drive::findObject(int sonarDistance){
   /* if (targetLat == 0 && targetLong == 0)
    {
        while(sonarDistance >= 200)
        {
            
            steer.write(LEFT);
            motor.write(go_forward);
            //delay(1000);
            //motor.write(STOP);
            //CheckUS();
        }
        while(dist < 200)
        {
            steer.write(STRAIGHT);
            motor.write(go_forward);
        }
    }*/
}
static void Drive::pressed(){
    switchPressed = true;
}

static void Drive::endlessLoop(){
    display.clear();
    display.print("Stopped",0,0);
    while (1)
    {
        motor.write(STOP);
        delay(1000);
    }
}
