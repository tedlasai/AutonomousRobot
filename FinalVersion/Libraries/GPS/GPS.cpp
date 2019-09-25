
#include "GPS.h"

GPS::GPS(){
    targetLat = 38.8937530;//38.963020324;
    targetLong = -104.8023580;
}
void GPS::init(){
    Serial1.begin(9600);
    display.clear();
    display.print("GPS Initializing",0,0);
    delay(1000);
    for(int i = 0; i < 10; i++){
        get();
        print();
        delay(100);
    }
}

void GPS::print()
{
    display.print("LAT:",0,0);
    display.print(currentLat,10);
    display.print("LON:",1,0);
    display.print(currentLong,10);
}

void GPS::get(){
    bool newdata = false;
    while(!newdata){
    unsigned long start = millis();
        // Every 0.5 seconds we print an update
        while (millis() - start < 500)
        {
            if (Serial1.available())
                
            {
                char c = Serial1.read();
                //Serial.print(c);  // uncomment to see raw GPS data
                if (_gps.encode(c))
                {
                    newdata = true;
                    break;  // uncomment to print new data immediately!
                }
            }
        }
        if (newdata)
        {
            _gps.f_get_position(&currentLat, &currentLong);
        }
    }
}

int GPS::calculateTargetHeading(){
    //determine course heading
        float dlon = radians(targetLong-currentLong);
        float cLat = radians(currentLat);
        float tLat = radians(targetLat);
        float a2=atan2( sin(dlon)*cos(tLat), cos(cLat)*sin(tLat)-sin(cLat)*cos(tLat)*cos(dlon));
        a2 = degrees(a2);
        if(a2<0){
            a2 +=360;
        }
    targetHeading = a2;
    return targetHeading;
}

void GPS::printTargetHeading(){
    display.print("T:",1,11); // Print a message to the LCD
    display.print(targetHeading); // Print a message to the LCD
    if(targetHeading<10){
        display.print("  ");
    } else if(targetHeading<100){
        display.print(" ");
    }
    delay(100);
}

float GPS::calculateDistance(){
    float R = 6371000; // metres
    float phi1 = radians(currentLat);
    float phi2 = radians(targetLat);
    float deltaPhi = radians(targetLat-currentLat);
    float deltaLambda = radians(targetLong-currentLong);
    
    float a = sin(deltaPhi/2) * sin(deltaPhi/2) +
    cos(phi1) * cos(phi2) *
    sin(deltaLambda/2) * sin(deltaLambda/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    targetDistance = R * c;
    Serial.println(targetDistance);
    return targetDistance;
}

void GPS::printDistance(){
    display.print("H:",1,0); // Print a message to the LCD
    display.print(targetDistance,4); // Print a message to the LCD
    delay(10);
}
