
#include "Display.h"


Display::Display():_lcd(DISPLAY_ADDRESS, DISPLAY_LENGTH, DISPLAY_LINES){
}

void Display::init(){
    _lcd.init(); //initialize the lcd
    _lcd.backlight(); //open the backlight
    _lcd.clear();//clear lcd
}


void Display::print(String input)
{
    _lcd.print(input);
}

void Display::print(String input, int line, int offset)
{
    _lcd.setCursor(offset,line); // Sets the location at which subsequent text written to the LCD will be displayed
    _lcd.print(input);
}

void Display::print(int input)
{
    _lcd.print(input);
}

void Display::print(int input, int precision)
{
    _lcd.print(input, precision);
}

void Display::print(int input, int line, int offset)
{
    _lcd.setCursor(offset,line); // Sets the location at which subsequent text written to the LCD will be displayed
    _lcd.print(input);
}

void Display::print(double input)
{
    _lcd.print(input);
}

void Display::print(double input, int precision)
{
    _lcd.print(input, precision);
}

void Display::print(double input, int line, int offset)
{
    _lcd.setCursor(offset,line); // Sets the location at which subsequent text written to the LCD will be displayed
    _lcd.print(input);
}

void Display::clear(){
    _lcd.clear();
}

void Display::print(int32_t input)
{
    _lcd.print(input);
}

void Display::print(int32_t input, int precision)
{
    _lcd.print(input, precision);
}

void Display::print(int32_t input, int line, int offset)
{
    _lcd.setCursor(offset,line); // Sets the location at which subsequent text written to the LCD will be displayed
    _lcd.print(input);
}



