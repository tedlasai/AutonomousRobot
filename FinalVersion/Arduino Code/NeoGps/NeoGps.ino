#include <NMEAGPS.h>

//======================================================================
//  Program: NMEAdistance.ino
//
//  Description:  Display distance from a base location.
//
//  Prerequisites:
//     1) NMEA.ino works with your device (correct TX/RX pins and baud rate)
//     2) GPS_FIX_LOCATION has been enabled.
//     3) A sentence that contains lat/long has been enabled (GGA, GLL or RMC).
//     4) Your device sends at least one of those sentences.
//
//  'Serial' is for debug output to the Serial Monitor window.
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

//------------------------------------------------------------
// Check that the config files are set up properly

#if !defined( NMEAGPS_PARSE_RMC ) &  \
    !defined( NMEAGPS_PARSE_GGA ) &  \
    !defined( NMEAGPS_PARSE_GLL )
  #error You must uncomment at least one of NMEAGPS_PARSE_RMC, NMEAGPS_PARSE_GGA or NMEAGPS_PARSE_GLL in NMEAGPS_cfg.h!
#endif

#if !defined( GPS_FIX_LOCATION )
  #error You must uncomment GPS_FIX_LOCATION in GPSfix_cfg.h!
#endif

NMEAGPS gps;

// The base location, in degrees * 10,000,000
NeoGPS::Location_t base( 388937530L, -1048023580L ); // WestLawn
/*NeoGPS::Location_t waypoints[] =
  {
    { 13062820, 1038322750 },
    { 13062820, 1038322750 },
    { 13023070, 1038368650 },
    { 13062820, 1038322750 },
    { 13065860, 1038328320 },
    { 12714200, 1038192080 },
    { 12480740, 1038213810 },
    { 12842550, 1038608650 },
    { 12894280, 1038465000 },
  };*/
void setup()
{
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.println( F("NMEAdistance.ino started.") );
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  gpsPort.begin(9600);

} // setup

void loop()
{
  while (gps.available( gpsPort )) {
    gps_fix fix = gps.read(); // save the latest

    // When we have a location, calculate how far away we are from the base location.
    if (fix.valid.location) {
      float distance = fix.location.DistanceKm( base ) * 1000 ;
      float bearing = fix.location.BearingToDegrees(base);
      DEBUG_PORT.print( F("Distance: ") );
      DEBUG_PORT.print( distance );
      DEBUG_PORT.println( F(" Meters") );
      DEBUG_PORT.print( F("Degrees: ") );
      DEBUG_PORT.print( bearing );
      DEBUG_PORT.println( F(" Degrees") );
    } else
      // Waiting...
      DEBUG_PORT.print( '.' );
  }
} // loop
