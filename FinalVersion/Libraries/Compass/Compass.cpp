
#include "Compass.h"

void Compass::init(){
    display.clear();
    display.print("Compass Initializing",0,0);
    delay(1000);
    if (_mag.begin() != 0)
    {
        display.print("Error Connection",0,0);
        display.print("Magnetometer",1,0);
        return;
    }
    
    // set the amount of gain
    //
    // LSB/Gauss   Field Range
    // 1370     +- 0.88 Ga
    // 1090     +- 1.3 Ga
    // 820      +- 1.9 Ga
    // 660      +- 2.5 Ga
    // 440      +- 4.0 Ga
    // 390      +- 4.7 Ga
    // 330      +- 5.6 Ga
    // 230      +- 8.1 Ga
    _mag.setGain(HMC5833L_GAIN_440);
}

void Compass::printRaw()
{
    // print them out
    display.print("X:",0,0);
    display.print(x);
    display.print("   Y:");
    display.print(y);
    display.print("  ");
    display.print("Z:",1,0);
    display.print(z);
    delay(50);
}

void Compass::print()
{
    display.print("C:",1,6); // Print a message to the LCD
    display.print(currentHeading); // Print a message to the LCD
    if(currentHeading<10){
        display.print("  ");
    } else if(currentHeading<100){
        display.print(" ");
    }
    delay(50);
}

int Compass::get(){
    int8_t ret = _mag.readRaw(&x,&y, &z);
    
    x = (x-X_MID)*(Y_MAX-Y_MIN)/(X_MAX -X_MIN);
    y = y -(Y_MID);
    
    double tan_heading = atan2(y, x);
    
    // Correct for when signs are reversed.
    if(tan_heading < 0)
        tan_heading += 2 * M_PI;
    
    tan_heading += DECLINATION_ANGLE;
    // convert to degrees
    currentHeading = tan_heading * 180 / M_PI;
    
    currentHeading = currentHeading +(COMPASS_OFFSET);
    while(currentHeading < 0){
        currentHeading += 360;
    }
    while(currentHeading > 360){
        currentHeading -= 360;
    }
    return currentHeading;
}

void Compass::getRaw(){
    int8_t ret = _mag.readRaw(&x,&y, &z);
}
