/*
 road location/segmentation
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

#ifndef ROAD_H
#define ROAD_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"
#include "vehiclegeometry.h"

void road_create_lookups(
    int image_width,
    int image_height,
    int min_range_mm,
    int max_range_mm,
    int camera_height_mm,
    int fov,
    int tilt,
    int vehicle_width_mm,
    int projected_width,
    int projected_height,
    int * projected_lookup,
    int * reprojected_lookup);

void road_segment(
    int image_width,
    int image_height,
    int sample_width,
    int threshold,
    unsigned char * image_data,
    unsigned char * image_data_buffer,
    unsigned char * image_segmented);

void road_project(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int projected_width,
    int projected_height,
    int * projected_lookup,
    unsigned char * projected);

void road_reproject(
    int projected_width,
    int projected_height,
    unsigned char * image_ground_plane,
    int image_width,
    int image_height,
    int * reprojected_lookup,
    unsigned char * image_data);

#endif
