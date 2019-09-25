#include <Display.h>


Display display;
void setup() 
{
  display.init();
}
void loop() 
{
  display.print("hi",2,3);
}

