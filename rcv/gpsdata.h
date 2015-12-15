/*
 Obtain data from a gps device
 The reason for wrapping gpsd here is to provide some abstraction such
 that potentially other gps interfaces could be implemented
 Copyright (C) 2010 Bob Mottram
 fuzzgun@gmail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GPSDATA_H
#define GPSDATA_H

#define GPSD_SERVER                       "localhost"
#define GPSD_PORT                         "2947"
#define GPS_POLL_FREQUENCY_MILLISECONDS   1000

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gps.h>

int gpsdata_start();
void gpsdata_stop();
void gpsdata_update(
    double * longitude,
    double * latitude,
    double * speed);
int gpsdata_miles_per_hour();
int gpsdata_kilometers_per_hour();

#endif /* GPSDATA_H */

