/*
 Traffic light detection
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

#ifndef TRAFFICLIGHTS_H
#define TRAFFICLIGHTS_H

/* number of events to retain in the history for online update */
#define TRAFFICLIGHTS_HISTORY                8
#define TRAFFICLIGHTS_HISTORY_COUNTER_MAX    10
#define TRAFFICLIGHTS_HISTORY_FIELDS         2
#define TRAFFICLIGHTS_HISTORY_FIELD_COUNTER  0
#define TRAFFICLIGHTS_HISTORY_FIELD_STATE    1

/* Maximum lights detected */
#define TRAFFICLIGHTS_MAX                    4

/* Number of thresholds to try (1 or 2) */
#define TRAFFICLIGHTS_THRESHOLDS             1

/* Fields defining a traffic light region */
#define TRAFFICLIGHTS_FIELDS                 5
#define TRAFFICLIGHTS_FIELD_TX               0
#define TRAFFICLIGHTS_FIELD_TY               1
#define TRAFFICLIGHTS_FIELD_BX               2
#define TRAFFICLIGHTS_FIELD_BY               3
#define TRAFFICLIGHTS_FIELD_COLOUR           4

/* Percent of the maximum red or green value used as a threshold */
#define TRAFFICLIGHTS_COLOUR_THRESHOLD       10
#define TRAFFICLIGHTS_RED_THRESHOLD          50
#define TRAFFICLIGHTS_GREEN_THRESHOLD        50

/* Threshold to check edge response on the left and
   right sides of a region */
#define TRAFFICLIGHTS_EDGES_THRESHOLD        20

/* How straight are the sides of the region? */
#define TRAFFICLIGHTS_EDGES_VARIANCE_THRESHOLD 10

/* Thresholds */
#define TRAFFICLIGHTS_REFLECTANCE_THRESHOLD1 300
#define TRAFFICLIGHTS_REFLECTANCE_THRESHOLD2 220
#define TRAFFICLIGHTS_VARIANCE_THRESHOLD1    35
#define TRAFFICLIGHTS_VARIANCE_THRESHOLD2    30

/* Expected aspect ratio range for traffic lights */
#define TRAFFICLIGHTS_ASPECT_MIN             130
#define TRAFFICLIGHTS_ASPECT_MAX             340
#define TRAFFICLIGHTS_ASPECT_MAX2            600

/* Index numbers used to represent light colours */
#define TRAFFICLIGHT_STATE_UNKNOWN           0
#define TRAFFICLIGHT_STATE_RED               1
#define TRAFFICLIGHT_STATE_AMBER             2
#define TRAFFICLIGHT_STATE_GREEN             3

/* Value used to represent traffic light colour when filtering */
#define TRAFFICLIGHT_COLOUR                  150

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "vehiclegeometry.h"
#include "floodfill.h"

int trafficlights_validate_edge_variance(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int layer,
    int tx,
    int ty,
    int bx,
    int by,
    int threshold);

int trafficlights_validate_edges(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    int threshold);

void trafficlights_filter_colours(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int * lookup,
    int horizon,
    int layer,
    unsigned short * lanes,
    int reflectance_threshold);

void trafficlights_filter_thin_verticals(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    int minimum_width);

void trafficlights_filter_thin_horizontals(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    int minimum_width);

int trafficlights_colour(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int tx,
    int ty,
    int bx,
    int by);

void trafficlights_update_region(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int layer,
    int * tx,
    int * ty,
    int * bx,
    int * by);

int trafficlights_validate(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned short * lanes,
    int * colour,
    int layer,
    int edge_threshold,
    int edge_variance_threshold);

int trafficlights_max_reflectance(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * lookup,
    int horizon,
    unsigned short * lanes,
    int reflectance_threshold,
    int variance_threshold);

void trafficlights_show_search_region(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * lookup,
    int horizon,
    unsigned short * lanes);

void trafficlights_filter(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int * lookup,
    int horizon,
    int layer,
    unsigned short * lanes,
    int reflectance_threshold,
    int variance_threshold);

int trafficlights_detect(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    unsigned short * stack,
    unsigned short * lanes,
    unsigned short * regions);

void trafficlights_show(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int no_of_traffic_lights,
    unsigned short * traffic_lights,
    int BGR);

int trafficlights_online_update(
    int lights_state,
    unsigned short * traffic_light_history,
    int minimum_observations);

#endif
