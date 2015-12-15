/*
 detection of road markings
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

#ifndef ROADMARKINGS_H
#define ROADMARKINGS_H

#include <string.h>
#include "common.h"

/* minimum slope of lane markings within the original image
   as a percentage of the image width */
#define ROADMARKINGS_MINIMUM_SLOPE  2

void draw_line(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    int r,
    int g,
    int b,
    int line_only);

int roadmarkings_ground_plane_line_is_vertical(
    int ground_plane_tx,
    int ground_plane_ty,
    int ground_plane_bx,
    int ground_plane_by,
    int image_width,
    int projected_width,
    int * projected_lookup);

int roadmarkings_line_fit(
    int no_of_points,
    unsigned short * points,
    int min_start_x,
    int max_start_x,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    unsigned short * curb,
    int image_width,
    int * projected_lookup);

void roadmarkings_reproject_line(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int projected_width,
    int projected_height,
    int ground_plane_min_range_mm,
    int ground_plane_max_range_mm,
    unsigned short * line,
    unsigned short * line_projected,
    int * projected_lookup,
    int line_range_mm);

void roadmarkings_reproject_lanes(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int projected_width,
    int projected_height,
    int ground_plane_min_range_mm,
    int ground_plane_max_range_mm,
    unsigned short * lanes,
    unsigned short * lanes_projected,
    int * projected_lookup,
    int line_range_mm);

void roadmarkings_draw_lanes(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned short * lanes_projected,
    int r, int g, int b,
    int line_width);

int roadmarkings_update_markings(
    int projected_width,
    int * no_of_points_left,
    unsigned short * points_left,
    int * no_of_points_right,
    unsigned short * points_right,
    int left_x,
    int right_x,
    int y,
    unsigned char * segmented,
    unsigned char * markings);

int roadmarkings_lane_probability(
    int projected_width,
    int projected_height,
    int lane_width,
    unsigned char * ground_plane,
    unsigned char * segmented,
    unsigned char * markings,
    int update_markings,
    int edge_threshold,
    int * no_of_points_left,
    unsigned short * points_left,
    int * no_of_points_right,
    unsigned short * points_right,
    int min_x,
    int max_x,
    int w0,
    int w1,
    int expansion,
    int tilt_expansion);

void roadmarkings_edges(
    int projected_width,
    int projected_height,
    unsigned char * ground_plane,
    unsigned char * ground_plane_edges);

int roadmarkings_detect(
    int projected_width,
    int projected_height,
    int vehicle_width_mm,
    int ground_plane_width_mm,
    int edge_threshold,
    unsigned char * ground_plane,
    unsigned char * segmented,
    unsigned char * markings,
    unsigned short * lanes,
    int line_threshold,
    int image_width,
    int * projected_lookup);

#endif
