/*
 Number images used to display the distance to a vehicle
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

#ifndef NUMBERIMAGES_H
#define NUMBERIMAGES_H

void numberimages_draw_decimal(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int numerator,
    int denominator,
    int decimal_places,
    int metres,
    int x,
    int y,
    int r, int g, int b);

#endif
