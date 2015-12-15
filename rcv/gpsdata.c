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

#include "gpsdata.h"

static struct gps_data_t *gpsdata;
static long int last_gps_time;

/* start the gps device */
int gpsdata_start()
{
    gpsdata = gps_open(GPSD_SERVER, GPSD_PORT);
    if (gpsdata) {
        (void)gps_stream(gpsdata, WATCH_ENABLE & POLL_NONBLOCK, NULL);
        return 1;
    }
    else {
        return 0;
    }
}

/* returns mph */
int gpsdata_miles_per_hour()
{
    if (gpsdata) {
        if (!isnan(gpsdata->fix.speed)) {
            return (int)(gpsdata->fix.speed*3600/1609);
        }
        else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

/* returns km/h */
int gpsdata_kilometers_per_hour()
{
    if (gpsdata) {
        if (!isnan(gpsdata->fix.speed)) {
            return (int)(gpsdata->fix.speed*18/5);
        }
        else {
            return 0;
        }
    }
    else {
        return 0;
    }
}

/* poll the gps device for new data
   longitude and latitude are in degrees
   speed is in metres per second */
void gpsdata_update(
    double * longitude,
    double * latitude,
    double * speed)
{
    int elapsed_mS;
    long int time;
    struct timeval t;

    if (gpsdata) {
        gettimeofday(&t, NULL);
        time = (long int)(t.tv_usec + 1000000 * t.tv_sec);
        elapsed_mS = (int)((time - last_gps_time)/1000);
        if (elapsed_mS < 0) elapsed_mS = -elapsed_mS;
        if (elapsed_mS > GPS_POLL_FREQUENCY_MILLISECONDS) {
            last_gps_time = time;
            (void)gps_poll(gpsdata);
            *longitude = gpsdata->fix.longitude;
            *latitude = gpsdata->fix.latitude;
            *speed = gpsdata->fix.speed;
        }
    }
}

/* stop listening to the gps device */
void gpsdata_stop()
{
    if (gpsdata) {
        (void)gps_close(gpsdata);
    }
}


