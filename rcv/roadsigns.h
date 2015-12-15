/*
 detection of road signs
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

#ifndef ROADSIGNS_H
#define ROADSIGNS_H

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "common.h"
#include "floodfill.h"
#include "png2.h"

#define ROADSIGNS_MAX_REGIONS             30

#define ROADSIGNS_MAX_TRIANGLE            64
#define ROADSIGNS_MAX_CIRCLE              32
#define ROADSIGNS_MAX_TRIANGLE_INVERTED   8
#define ROADSIGNS_MAX_SQUARE              8

#define ROADSIGNS_SHAPES                  4
#define ROADSIGNS_SHAPE_TRIANGLE          0
#define ROADSIGNS_SHAPE_CIRCLE            1
#define ROADSIGNS_SHAPE_TRIANGLE_INVERTED 2
#define ROADSIGNS_SHAPE_SQUARE            3

#define ROADSIGNS_DESCRIPTOR_SIZE         32
#define ROADSIGNS_DESCRIPTOR_ENTRIES      6
#define ROADSIGNS_SHAPE_TEMPLATE_SIZE     16

#define ROADSIGNS_TEMPLATE_RED            1
#define ROADSIGNS_TEMPLATE_WHITE          2
#define ROADSIGNS_TEMPLATE_BLACK          3

int roadsigns_regions_join(
    unsigned short * regions,
    int no_of_regions,
    int min_overlap);

void roadsigns_vertices(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    int * vertices);
    
void roadsigns_descriptor(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int shape_index,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned char * descriptor);

void roadsigns_update_eigen_descriptors(
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors);

void roadsigns_match_descriptor(
    unsigned char * test_descriptor,
    int shape_index,
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int ** sign_matches);
  
void roadsigns_identify_sign(
    int image_width,
    int image_height,
    unsigned char * image_road_signs,
    int tx,
    int ty,
    int bx,
    int by,
    int shape_index,
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int ** sign_matches);

void roadsigns_colour_model(
    int no_of_examples,
    unsigned char * examples,
    int image_width,
    int image_height,
    unsigned char * image_sign_colours,
    unsigned char * model);

void roadsigns_colour_models(
    int image_width,
    int image_height,
    unsigned char * image_sign_colours);
        
void roadsigns_recognise(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_road_signs,
    int tx,
    int ty,
    int bx,
    int by,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int * shape_index,
    int ** sign_matches);

void roadsigns_analyse_regions(
    int image_width,
    int image_height,
    int horizon,
    unsigned char * image_data,
    unsigned char * image_road_signs,
    int r,
    int g,
    int b,
    int filled_r,
    int filled_g,
    int filled_b,
    int minimum_sign_width,
    int maximum_sign_width,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int * shape_index,
    int * sign_index,
    int ** sign_matches,
    int * tx,
    int * ty,
    int * bx,
    int * by,
    unsigned short * stack,
    unsigned short * regions);

void roadsigns_detect(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int horizon,
    int minimum_sign_width,
    int maximum_sign_width,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    unsigned char * model,
    unsigned char * image_road_signs,
    int * shape_index,
    int * sign_index,
    int ** sign_matches,
    int * tx,
    int * ty,
    int * bx,
    int * by,
    unsigned short * stack,
    unsigned short * regions);

void roadsigns_create_shapes(
    unsigned char * sign_shapes);

int roadsigns_identify_shape(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned char * sign_shapes);
    
void roadsigns_load_database(
    char * signs_database,
    int * no_of_signs,
    char *** sign_text,
    unsigned char * sign_shapes,
    unsigned char ** sign_descriptor,
    unsigned char ** sign_eigen_descriptor);

void roadsigns_show(
    int image_width,
    unsigned char * image_data,
    int shape_index,    
    int tx,
    int ty,
    int bx,
    int by);
        
#endif
