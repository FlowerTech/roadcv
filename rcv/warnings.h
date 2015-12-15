/*
 Warnings of various kinds
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

#ifndef WARNINGS_H
#define WARNINGS_H

 /* various warning flags */
#define WARNING_STOPPING_DISTANCE        1
#define WARNING_LANE_CROSSING            2
#define WARNING_SPEED_LIMIT              4
#define WARNING_STOP_LIGHT               8
#define WARNING_GO                       16

/* minimum time interval between audible warnings */
#define WARNING_TIME_INTERVAL_MS         5000

/* filenames for sounds */
#define WARNING_SOUND_STOPPING_DISTANCE  "stopping_distance.au"
#define WARNING_SOUND_LANE_CROSSING      "lane_crossing.au"
#define WARNING_SOUND_SPEED_LIMIT        "speed_limit.au"
#define WARNING_SOUND_STOP_LIGHT         "stop_light.au"
#define WARNING_SOUND_GO                 "go.au"

/* This can be customised to your preferred sound player utility */
#define SOUND_COMMAND                    "aplay"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "common.h"

int warnings_play_sound(
    char * sounds_path,
    const char * sound_filename);
void warnings_audible(
    char * sounds_path,
    int flags,
    long int * last_warning_time);
void warnings_show_icons(
    int flags,
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * current_warning_icon,
    long int * last_warning_icon_time);
void warnings_draw_circle_icon(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int r,
    int g,
    int b);

#endif

