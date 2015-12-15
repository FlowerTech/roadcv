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

#include "numberimages.h"

static unsigned char number_icon[] = {
/* 0 */
    0,0,0,1,1,0,0,0,
    0,0,1,0,0,1,0,0,
    0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,
    0,0,1,0,0,1,0,0,
    0,0,0,1,1,0,0,0,
/* 1 */
    0,0,0,0,1,0,0,0,
    0,0,0,1,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,1,1,1,0,0,
/* 2 */
    0,0,1,1,1,0,0,0,
    0,1,0,0,0,1,0,0,
    0,1,0,0,0,1,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,0,
    0,0,1,0,0,0,0,0,
    0,1,0,0,0,0,0,0,
    0,1,1,1,1,1,0,0,
/* 3 */
    0,0,1,1,1,0,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,1,1,1,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,1,1,1,0,0,0,
/* 4 */
    0,0,0,0,0,1,0,0,
    0,0,0,0,1,1,0,0,
    0,0,0,1,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,1,0,0,0,1,0,0,
    0,1,1,1,1,1,1,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
/* 5 */
    0,0,1,1,1,1,0,0,
    0,0,1,0,0,0,0,0,
    0,0,1,0,0,0,0,0,
    0,0,1,1,1,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,1,0,0,0,
    0,0,1,1,0,0,0,0,
/* 6 */
    0,0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,0,
    0,0,1,0,0,0,0,0,
    0,0,1,1,1,0,0,0,
    0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,0,1,1,0,0,0,
/* 7 */
    0,0,1,1,1,1,1,0,
    0,0,0,0,0,0,1,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,0,
    0,0,0,1,0,0,0,0,
/* 8 */
    0,0,0,1,1,0,0,0,
    0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,0,1,1,0,0,0,
    0,0,0,1,1,0,0,0,
    0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,0,1,1,0,0,0,
/* 9 */
    0,0,0,1,1,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,1,0,0,1,0,0,
    0,0,0,1,1,1,0,0,
    0,0,0,0,0,1,0,0,
    0,0,0,0,1,0,0,0,
    0,0,0,1,0,0,0,0,
    0,0,1,0,0,0,0,0,
/* decimal */
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,1,1,0,0,0,
    0,0,0,1,1,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
/* m */
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,1,0,0,1,0,0,
    0,1,0,1,1,0,1,0,
    0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0,
    0,1,0,0,0,0,1,0
};

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
    int r, int g, int b)
{
    int div,n,xx,yy,i,index,min,max,mult=1;

    if ((y<0) || (y >= image_height-9)) return;

    n=numerator/(denominator*mult);
    while (n > 0) {
        n = numerator/(denominator*mult);
        if (n>0) mult*=10;
    }
    if (numerator<denominator) mult*=10;
    max = denominator*mult/10;
    min = denominator/(10*decimal_places);

    for (div=max;div>=min;div/=10) {
        n = numerator/div;
        if ((x > -1) && (x < image_width-9)) {
            index = n*8*8;
            for (yy=0;yy<8;yy++) {
                for (xx = 0; xx < 8; xx++,index++) {
                    if (number_icon[index]!=0) {
                        i=((y+yy)*image_width+xx+x)*3;
                        image_data[i] = r;
                        image_data[i+1] = g;
                        image_data[i+2] = b;
                    }
                }
            }
        }
        if (div==denominator) {
            x+=8;
            if ((x > -1) && (x < image_width-9)) {
                index = 10*8*8;
                for (yy=0;yy<8;yy++) {
                    for (xx = 0; xx < 8; xx++,index++) {
                        if (number_icon[index]!=0) {
                            i=((y+yy)*image_width+xx+x)*3;
                            image_data[i] = r;
                            image_data[i+1] = g;
                            image_data[i+2] = b;
                        }
                    }
                }
            }
        }
        numerator -= n*div;
        x+=8;
    }

    if (metres!=0) {
        if ((x > -1) && (x < image_width-9)) {
            index = 11*8*8;
            for (yy=0;yy<8;yy++) {
                for (xx = 0; xx < 8; xx++,index++) {
                    if (number_icon[index]!=0) {
                        i=((y+yy)*image_width+xx+x)*3;
                        image_data[i] = r;
                        image_data[i+1] = g;
                        image_data[i+2] = b;
                    }
                }
            }
        }
    }
}

