
#include "Compass.h"
#include "Display.h"

Display display;
Compass compass;

void setup()
{
 
    compass.init();
    display.init();
 
}

void loop()
{
    // don't bother reading if we failed to connect
    //compass.getRaw();
    //compass.printRaw();
    compass.get();
    compass.print();
}

