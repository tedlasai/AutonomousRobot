#include <NMEAGPS.h>
#include <NMEAGPS.h>
#include <GPSport.h>

//======================================================================
//  Program: NMEA_isr.ino
//
//  Prerequisites:
//     1) NMEA.ino works with your device
//
//  Description:  This minimal program parses the GPS data during the 
//     RX character interrupt.  The ISR passes the character to
//     the GPS object for parsing.  The GPS object will add gps_fix 
//     structures to a buffer that can be later read() by loop().
//
//  License:
//    Copyright (C) 2014-2017, SlashDevin
//
//    This file is part of NeoGPS
//
//    NeoGPS is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    NeoGPS is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.
//
//======================================================================

#include <GPSport.h>

#include <Streamers.h>

// Check configuration
#include "Display.h"
Display display;

#ifndef NMEAGPS_INTERRUPT_PROCESSING
  #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

static NMEAGPS   gps;
gps_fix fix;                   // The GPS object
static void GPSisr( uint8_t c )
{
  gps.handle( c );

} // GPSisr

//--------------------------
NeoGPS::Location_t  base( 388932432L, -1048022577L );
void setup()
{
  NeoSerial.begin(9600);
  DEBUG_PORT.begin(9600);
  while (!DEBUG_PORT)
    ;

  DEBUG_PORT.print( F("NMEA_isr.INO: started\n") );
  DEBUG_PORT.print( F("fix object size = ") );
  DEBUG_PORT.println( sizeof(gps.fix()) );
  DEBUG_PORT.print( F("NMEAGPS object size = ") );
  DEBUG_PORT.println( sizeof(gps) );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  trace_header( DEBUG_PORT );

  DEBUG_PORT.flush();

  gpsPort.attachInterrupt( GPSisr );
  gpsPort.begin( 9600 );
  display.init();
}

//--------------------------

void loop()
{
  if (gps.available()) {
    fix = gps.read();
    trace_all( DEBUG_PORT, gps, gps.read() );
    
    double targetDistance = fix.location.DistanceKm( base ) * 1000;

    display.print("H:",0,0); // Print a message to the LCD
    display.print(targetDistance,4); // Print a message to the LCD
    delay(50);
    DEBUG_PORT.println(targetDistance);
  }

  if (gps.overrun()) {
    gps.overrun( false );
    DEBUG_PORT.println( F("DATA OVERRUN: took too long to print GPS data!") );
  }
}

