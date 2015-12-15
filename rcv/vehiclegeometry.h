/*
 vehicle related geometry
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

#ifndef VEHICLE_GEOMETRY_H
#define VEHICLE_GEOMETRY_H

#include <math.h>

#define GEOMETRY_INVALID  99999

void geometry_create_height_lookup(
    int image_width,
    int image_height,
    int * lookup,
    int min_height_mm,
    int max_height_mm,
    int width_mm,
    int horizon,
    int camera_height_mm,
    int fov,
    int tilt);

int geometry_vertical_range(
    int image_width,
    int image_height,
    int x,
    int * lookup,
    int horizon,
    unsigned short * lanes,
    int * y_bottom,
    int * y_top);

int geometry_pixel_distance(
    int y,
    int camera_height_mm,
    int fov,
    int image_width,
    int image_height,
    int tilt);

int geometry_pixel_right(
    int x,
    int distance_mm,
    int fov,
    int image_width);

int geometry_distance_forward_pixel(
    int distance_mm,
    int camera_height_mm,
    int observed_point_height_mm,
    int fov,
    int image_width,
    int image_height,
    int tilt);

int geometry_distance_right_pixel(
    int distance_mm,
    int baseline_mm,
    int fov,
    int image_width);

void geometry_show_distances(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int camera_height_mm,
    int fov,
    int tilt,
    int min_distance_mm,
    int max_distance_mm,
    int distance_increment_mm,
    int camera_index,
    int baseline_mm,
    int road_width_mm,
    int offset_from_road_centre_mm);

void geometry_create_range_lookup(
    int image_width,
    int image_height,
    int * lookup,
    int horizon,
    int camera_height_mm,
    int fov,
    int tilt,
    int min_width_mm,
    int max_width_mm);

#endif

