/*
 flood fill,
 based on http://www.student.kuleuven.be/~m0216922/CG/floodfill.html
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

#include "floodfill.h"

int floodfill_pop(
    int * x,
    int * y,
    int image_width,
    unsigned short * stack,
    int * stackptr)
{
    if (*stackptr > 0) {
        *x = (int)stack[*stackptr];
        *y = (int)stack[*stackptr+1];
        *stackptr-=2;
        return 1;
    }
    else {
        return 0;
    }
}

int floodfill_push(
    int x, int y,
    int image_width,
    int * stackptr,
    unsigned short * stack,
    int stack_size)
{
    if (*stackptr < stack_size - 2) {
        *stackptr+=2;
        stack[*stackptr] = (unsigned short)x;
        stack[*stackptr+1] = (unsigned short)y;
        return 1;
    }
    else {
        return 0;
    }
}

/* replaces one colour with another */
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
    int * by)
{
    int n,i,stackptr=0;

    *tx = image_width;
    *ty = image_height;
    *bx = 0;
    *by = 0;

    if (floodfill_push(
        x, y, image_width, &stackptr, stack, stack_size)==0) return;

    while (floodfill_pop(&x, &y, image_width, stack, &stackptr)!=0) {
        if (x < *tx) *tx = x;
        if (y < *ty) *ty = y;
        if (x > *bx) *bx = x;
        if (y > *by) *by = y;

        i = y*image_width+x;
        n = i*3;
        image_data[n] = filled_r;
        image_data[n+1] = filled_g;
        image_data[n+2] = filled_b;
        if ((x + 1 < image_width) &&
            (image_data[n+3] == r) &&
            (image_data[n+4] == g) &&
            (image_data[n+5] == b)) {
            if (floodfill_push(
                x + 1, y,
                image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (image_data[(i-1)*3] == r) &&
            (image_data[(i-1)*3+1] == g) &&
            (image_data[(i-1)*3+2] == b)) {
            if (floodfill_push(
                x - 1, y, image_width,
                &stackptr, stack, stack_size)==0) break;
        }
        if ((y + 1 < image_height) &&
            (image_data[(i+image_width)*3] == r) &&
            (image_data[(i+image_width)*3+1] == g) &&
            (image_data[(i+image_width)*3+2] == b)) {
            if (floodfill_push(
                x, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((y - 1 >= 0) &&
            (image_data[(i-image_width)*3] == r) &&
            (image_data[(i-image_width)*3+1] == g) &&
            (image_data[(i-image_width)*3+2] == b)) {
            if (floodfill_push(
                x, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y + 1 < image_height) &&
            (image_data[(i+image_width+1)*3] == r) &&
            (image_data[(i+image_width+1)*3+1] == g) &&
            (image_data[(i+image_width+1)*3+2] == b)) {
            if (floodfill_push(
                x + 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y - 1 >= 0) &&
            (image_data[(i-image_width+1)*3] == r) &&
            (image_data[(i-image_width+1)*3+1] == g) &&
            (image_data[(i-image_width+1)*3+2] == b)) {
            if (floodfill_push(
                x + 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y + 1 < image_height) &&
            (image_data[(i+image_width-1)*3] == r) &&
            (image_data[(i+image_width-1)*3+1] == g) &&
            (image_data[(i+image_width-1)*3+2] == b)) {
            if (floodfill_push(
                x - 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y - 1 >= 0) &&
            (image_data[(i-image_width-1)*3] == r) &&
            (image_data[(i-image_width-1)*3+1] == g) &&
            (image_data[(i-image_width-1)*3+2] == b)) {
            if (floodfill_push(
                x - 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
    }
}


/* only apply flood fill to a particular RGB layer of the image */
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
    int * by)
{
    int n,i,stackptr=0;

    *tx = image_width;
    *ty = image_height;
    *bx = 0;
    *by = 0;

    if (floodfill_push(
        x, y, image_width, &stackptr,
        stack, stack_size)==0) return;

    while (floodfill_pop(&x, &y, image_width, stack, &stackptr)!=0) {
        if (x < *tx) *tx = x;
        if (y < *ty) *ty = y;
        if (x > *bx) *bx = x;
        if (y > *by) *by = y;

        i = y*image_width+x;
        n = i*3;
        image_data[n+layer] = filled;
        if ((x + 1 < image_width) &&
            (image_data[n+3+layer] == unfilled)) {
            if (floodfill_push(
                x + 1, y, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (image_data[(i-1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x - 1, y, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((y + 1 < image_height) &&
            (image_data[(i+1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((y - 1 >= 0) &&
            (image_data[(i-image_width)*3+layer] == unfilled)) {
            if (floodfill_push(
                x, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y + 1 < image_height) &&
            (image_data[(i+image_width+1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x + 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y - 1 >= 0) &&
            (image_data[(i-image_width+1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x + 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y + 1 < image_height) &&
            (image_data[(i+image_width-1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x - 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y - 1 >= 0) &&
            (image_data[(i-image_width-1)*3+layer] == unfilled)) {
            if (floodfill_push(
                x - 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
    }
}


/* A bitwise version of flood fill */
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
    int * by)
{
    int n,stackptr=0;
    int unfilled_bit = 1<<unfilled_bit_index;
    int filled_bit = 1<<filled_bit_index;
    int widthsub = image_width-1;
    int widthadd = image_width+1;

    *tx = image_width;
    *ty = image_height;
    *bx = 0;
    *by = 0;

    if (floodfill_push(
        x, y, image_width, &stackptr,
        stack, stack_size)==0) return;

    while (floodfill_pop(&x, &y, image_width, stack, &stackptr)!=0) {
        if (x < *tx) *tx = x;
        if (y < *ty) *ty = y;
        if (x > *bx) *bx = x;
        if (y > *by) *by = y;

        n = y*image_width+x;
        image_binary[n] |= filled_bit;
        if ((x + 1 < image_width) &&
            (image_binary[n+1] & unfilled_bit)) {
            if (floodfill_push(
                x + 1, y, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (image_binary[n-1] & unfilled_bit)) {
            if (floodfill_push(
                x - 1, y, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((y + 1 < image_height) &&
            (image_binary[n+image_width] & unfilled_bit)) {
            if (floodfill_push(
                x, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((y - 1 >= 0) &&
            (image_binary[n-image_width] & unfilled_bit)) {
            if (floodfill_push(
                x, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y + 1 < image_height) &&
            (image_binary[n+widthadd] & unfilled_bit)) {
            if (floodfill_push(
                x + 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x + 1 < image_width) &&
            (y - 1 >= 0) &&
            (image_binary[n-widthadd] & unfilled_bit)) {
            if (floodfill_push(
                x + 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y + 1 < image_height) &&
            (image_binary[n+widthsub] & unfilled_bit)) {
            if (floodfill_push(
                x - 1, y + 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
        if ((x - 1 >= 0) &&
            (y - 1 >= 0) &&
            (image_binary[n-widthsub] & unfilled_bit)) {
            if (floodfill_push(
                x - 1, y - 1, image_width, &stackptr,
                stack, stack_size)==0) break;
        }
    }
}

