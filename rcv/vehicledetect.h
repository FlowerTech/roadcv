/*
 detection of road vehicles
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

#ifndef VEHICLE_DETECT_H
#define VEHICLE_DETECT_H

#include <stdio.h>
#include <string.h>
#include "common.h"
#include "floodfill.h"
#include "numberimages.h"

/* The maximum number of possible vehicles detected */
#define MAX_POSSIBLE_VEHICLES                  10

/* maximum horizontal displacement when tracking
   vehicles online */
#define VEHICLES_HORIZONTAL_TOLLERANCE         10

/* Number of fields for possible detected vehicles data */
#define POSSIBLE_VEHICLES_FIELDS               6

/* The maximum slope angle beyond which a region will
   not be considered as a vehicle */
#define MAX_SLOPE                              6

/* Threshold to determine whether a region is more than just a thin line */
#define MIN_AVERAGE_HEIGHT                     2

/* number of thresholds to try during shadow detection */
#define SHADOW_THRESHOLDS                      2

/* Number of frames over which to store possible vehicle data */
#define POSSIBLE_VEHICLES_HISTORY              20

/* Number of fields in the possible vehicles history */
#define POSSIBLE_VEHICLES_HISTORY_FIELDS       3

/* The minimum number of observations to be made before a vehicle is
   considered to have been positively detected */
#define POSSIBLE_VEHICLES_MIN_OBSERVATIONS     2

/* Field indexes within the possible vehicles history */
#define POSSIBLE_VEHICLES_HISTORY_FIELD_TIME   0
#define POSSIBLE_VEHICLES_HISTORY_FIELD_RANGE  1
#define POSSIBLE_VEHICLES_HISTORY_FIELD_CX     2

/* Field indexes for possible vehicle detections */
#define VEHICLES_FIELD_TX                      0
#define VEHICLES_FIELD_TY                      1
#define VEHICLES_FIELD_BX                      2
#define VEHICLES_FIELD_BY                      3
#define VEHICLES_FIELD_RANGE                   4
#define VEHICLES_FIELD_VALID                   5

/* Line fitting tollerance when calculating time to impact */
#define TTI_TOLLERANCE_MM                      1000

#define BASELINE_TIME_STEPS                    (POSSIBLE_VEHICLES_HISTORY>>1)
#define BASELINE_MILLISECONDS                  50

/* edge threshold used to determine if the sides of a region
   contain vertically oriented edges */
#define VEHICLEDETECT_SIDE_EDGES               10

/* If the percentage of pixels classified as road is higher than
   this value then the region is probably not a vehicle */
#define VEHICLEDETECT_MIN_REGION_VEHICLE_PERCENT  28

int vehicledetect_validate_region(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    int * reprojected_lookup,
    int tx,
    int ty,
    int bx,
    int by,
    int region_threshold);

int vehicledetect_validate_vehicle_sides(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int tx,
    int ty,
    int bx,
    int by,
    int vehicle_sides_threshold);

int vehicledetect_validate(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    int * reprojected_lookup,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int tx,
    int ty,
    int bx,
    int by,
    int slope_tollerance,
    int vehicle_sides_threshold,
    int region_threshold,
    int layer,
    int range_mm,
    int max_range_mm);

void vehicledetect_shadows(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int ty,
    int by,
    unsigned short * road_lines,
    int * reprojected_lookup,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    unsigned char * image_shadows,
    int layer,
    int reflectance_threshold_percent);

int vehicledetect_shadow_regions(
    int image_width,
    int image_height,
    unsigned char * segmented,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int ground_plane_width_mm,
    int vehicle_width_mm,
    int min_range_mm,
    int max_range_mm,
    int vehicle_max_range_ahead_mm,
    int * reprojected_lookup,
    int projected_width,
    int projected_height,
    int * possible_vehicles,
    unsigned short * stack,
    int layer,
    int horizon,
    int bonnet,
    int * range_lookup);

void vehicledetect_show_vehicles(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int r, int g, int b);

void vehicledetect_online_update(
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int * possible_vehicles_history,
    int time_interval_mS);

int vehicledetect_online_validation(
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int * possible_vehicles_history,
    int range_tollerance_mm);

void vehicledetect_add_observation(
    int * possible_vehicles,
    int index,
    int tx,
    int ty,
    int bx,
    int by,
    int range_mm);

void vehicledetect_shuffle_history(
    int * possible_vehicles_history);

int vehicledetect_stopping_distance(
    int speed_mph);

#endif
