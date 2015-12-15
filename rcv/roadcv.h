/*
 roadcv: computer vision for road vehicles
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

#ifndef ROADCV_H
#define ROADCV_H

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <omp.h>
#include "png2.h"
#include "floodfill.h"
#include "vehiclegeometry.h"
#include "road.h"
#include "roadmarkings.h"
#include "trafficlights.h"
#include "roadsigns.h"
#include "warnings.h"
#include "vehicledetect.h"

#define LANES_VALID_FRAMES 3
#define GEOCOORD_MULT      100000

struct rcv_data {
    int headless;
    int BGR;
    unsigned char * image_data;
    unsigned char * image_vehicles;
    unsigned char * image_ground_plane;
    unsigned char * image_ground_plane_buffer;
    unsigned char * image_segmented;
    unsigned char * image_road_markings;
    unsigned char * image_road_signs;
    unsigned char * image_traffic_lights;
    int no_of_traffic_lights[2];
    unsigned short * traffic_lights[2];
    unsigned char traffic_light_state;
    unsigned short * traffic_light_history;
    int * height_lookup;
    int min_road_sign_height_mm;
    int max_road_sign_height_mm;
    int traffic_light_width_mm;
    int * projected_lookup;
    int * reprojected_lookup;
    int * range_lookup;
    int image_width;
    int image_height;
    int projected_width;
    int projected_height;
    int fov;
    int tilt;
    int camera_height_mm;
    int vehicle_width_mm;
    int min_distance_mm;
    int max_distance_mm;
    int vehicle_max_range_ahead_mm;
    int vehicle_current_range_ahead_mm;
    int stopping_distance_mm;
    int speed_mph;
    int sample_width;
    int segmentation_threshold;
    int road_markings_edge_threshold;
    int road_markings_line_threshold;
    int vehicle_range_ahead_mm, ground_plane_width_mm;
    unsigned short road_lines[16];
    unsigned short lanes_online[8];
    int lane_displacement_threshold;
    int lanes_valid;
    int horizon;
    int bonnet;
    int minimum_sign_width;
    int maximum_sign_width;
    int minimum_symmetry;
    int sign_type;
    int shape_index;
    int sign_index;
    int sign_region[4];
    int * sign_matches[ROADSIGNS_SHAPES];
    unsigned char * sign_descriptor[ROADSIGNS_SHAPES];
    unsigned char * sign_eigen_descriptor[ROADSIGNS_SHAPES];
    int no_of_signs[ROADSIGNS_SHAPES];
    char ** sign_text[ROADSIGNS_SHAPES];
    unsigned char * sign_shapes;
    unsigned short * stack_traffic_lights[2];
    unsigned short * stack[SHADOW_THRESHOLDS+1];
    unsigned short * regions;
    int no_of_possible_vehicles[SHADOW_THRESHOLDS];
    int * possible_vehicles[SHADOW_THRESHOLDS];
    int * possible_vehicles_history;
    int range_tollerance_mm;
    int time_interval_mS;
    long int time;
    int warnings;
    long int last_warning_time;
    long int last_warning_icon_time;
    int current_warning_icon;
    int minimum_lane_crossing_warning_speed_mph;
    int lane_crossing_tollerance_percent;
    int latitude, longitude;
    char * sounds_path;
    unsigned char online;
    unsigned char show_lanes;
    unsigned char detect_vehicles;
    unsigned char detect_signs;
    unsigned char detect_traffic_lights;
    unsigned char enable_audio;
};

void roadcv_open(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * image_data,
    int BGR,
    int fov,
    int tilt,
    int camera_height_mm,
    int detect_vehicles,
    int detect_signs,
    int detect_traffic_lights,
    int enable_audio,
    char * sounds_path,
    char * signs_database,
    struct rcv_data * r,
    int online,
    int headless,
    int show_lanes,
    int minimum_lane_crossing_warning_speed_mph,
    int lane_crossing_tollerance_percent);
void roadcv_close(struct rcv_data * r);
void roadcv_markings(struct rcv_data * r);
void roadcv_signs(struct rcv_data * r);
void roadcv_BGRtoRGB(
    int image_width,
    int image_height,
    unsigned char * image_data);
void roadcv_update(struct rcv_data * r);
void roadcv_debug(struct rcv_data * r, int update_colour_model);
void roadcv_draw_lanes(struct rcv_data * r);
int roadcv_time_elapsed_mS(struct rcv_data * r);
void roadcv_vehicles(struct rcv_data * r);
void roadcv_lane_crossing_warning(struct rcv_data * r);
void roadcv_lights(struct rcv_data * r);

#endif
