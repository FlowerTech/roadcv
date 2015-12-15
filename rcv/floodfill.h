/*
 flood fill, based on
 http://www.student.kuleuven.be/~m0216922/CG/floodfill.html
 but without the divisions
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

#ifndef FLOODFILL_H
#define FLOODFILL_H

int floodfill_pop(
    int * x,
    int * y,
    int image_width,
    unsigned short * stack,
    int * stackptr);

int floodfill_push(
    int x, int y,
    int image_width,
    int * stackptr,
    unsigned short * stack,
    int stack_size);

void floodfill_fill(
    int x,
    int y,
    int image_width,
    int image_height,
    unsigned short * stack,
    int stack_size,
    unsigned char * image_data,
    unsigned char r,
    unsigned char g,
    unsigned char b,
    unsigned char filled_r,
    unsigned char filled_g,
    unsigned char filled_b,
    int * tx,
    int * ty,
    int * bx,
    int * by);

void floodfill_fill_layer(
    int x,
    int y,
    int image_width,
    int image_height,
    unsigned short * stack,
    int stack_size,
    unsigned char * image_data,
    int layer,
    unsigned char unfilled,
    unsigned char filled,
    int * tx,
    int * ty,
    int * bx,
    int * by);

void floodfill_fill_bitwise(
    int x,
    int y,
    int image_width,
    int image_height,
    unsigned short * stack,
    int stack_size,
    unsigned char * image_binary,
    unsigned char unfilled_bit_index,
    unsigned char filled_bit_index,
    int * tx,
    int * ty,
    int * bx,
    int * by);

#endif
