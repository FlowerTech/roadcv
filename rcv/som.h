/*
 self organising map
 This is used for classifying binary patterns - typically binary images
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

#ifndef _SOM_H_
#define _SOM_H_

#include <stdlib.h>
#include "common.h"

void som_draw(
    unsigned char * img, int img_width, int img_height,
    unsigned char * map, unsigned char * input);
unsigned char som_cycle(
    unsigned char * map, unsigned char * input, int reinforcement);
void som_init(
    unsigned char * map, unsigned char dim, unsigned char radius,
    unsigned char input_bits, int seed);

#endif

