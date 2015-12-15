/*
 Traffic light detection
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

#include "trafficlights.h"

/* Estimates the dark threshold by finding the peak response
   of dark pixels below a certain threshold */
int trafficlights_max_reflectance(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * lookup,
    int horizon,
    unsigned short * lanes,
    int reflectance_threshold,
    int variance_threshold)
{
    int x, y, n, y_bottom=0, y_top=0,found,variance,index,max=0;
    int r,g,b,reflectance=-1;
    int stride = image_width*3;
    int histogram[256];

    if (lanes[0]==0) return -1;
    memset((void*)histogram,'\0',256*sizeof(int));

    reflectance_threshold*=3;

    for (x = 0; x < image_width; x++) {
        found=0;
        if (geometry_vertical_range(
            image_width, image_height, x,
            lookup, horizon, lanes,
            &y_bottom, &y_top)!=0) {
            if (geometry_vertical_range(
                image_width, image_height, x,
                lookup, horizon, &(lanes[4]),
                &y_bottom, &y_top)==0) {
                found=2;
            }
        }
        else {
            found=1;
        }

        if (found!=0) {
            n = (y_top*image_width+x)*3;
            for (y=y_top;y<y_bottom;y++,n+=stride) {
                r = image_data[n]+image_data[n+3]+image_data[n+stride];
                g = image_data[n+1]+image_data[n+4]+image_data[n+stride+1];
                b = image_data[n+2]+image_data[n+5]+image_data[n+stride+2];
                if (r+g+b < reflectance_threshold) {
                    variance = ABS(r-b) + ABS(r-g) + ABS(g-b);
                    if ((ABS(r-b) < variance_threshold) &&
                        (ABS(r-g) < variance_threshold) &&
                        (ABS(g-b) < variance_threshold)) {
                        index = (r+g+b)/9;
                        if (index > 0) {
                            histogram[index-1]++;
                            histogram[index]+=2;
                            histogram[index+1]++;
                        }
                    }
                }
            }
        }
    }
    for (n=1;n<256;n++) {
        if (histogram[n]>max) {
            max=histogram[n];
            reflectance=n;
        }
    }
    return reflectance;
}

/* Draws bounding boxes around detected traffic lights.
   The colour of the box represents the light state. */
void trafficlights_show(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int no_of_traffic_lights,
    unsigned short * traffic_lights,
    int BGR)
{
    int i,x,y,n,tx,ty,bx,by,colour,r,g,b;
    int stride = image_width*3;

    for (i=0;i<no_of_traffic_lights;i++) {
        tx = traffic_lights[i*TRAFFICLIGHTS_FIELDS+
            TRAFFICLIGHTS_FIELD_TX];
        ty = traffic_lights[i*TRAFFICLIGHTS_FIELDS+
            TRAFFICLIGHTS_FIELD_TY];
        bx = traffic_lights[i*TRAFFICLIGHTS_FIELDS+
            TRAFFICLIGHTS_FIELD_BX];
        by = traffic_lights[i*TRAFFICLIGHTS_FIELDS+
            TRAFFICLIGHTS_FIELD_BY];
        colour = traffic_lights[i*TRAFFICLIGHTS_FIELDS+
            TRAFFICLIGHTS_FIELD_COLOUR];
        r=g=b=0;
        switch(colour) {
            case TRAFFICLIGHT_STATE_RED: { r=255; break; }
            case TRAFFICLIGHT_STATE_AMBER: { r=255; g=200; break; }
            case TRAFFICLIGHT_STATE_GREEN: { g=255; break; }
        }
        if (BGR!=0) {
            n = r;
            r = b;
            b = n;
        }
        for (y=ty;y<=by;y++) {
            n = (y*image_width+tx)*3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n-=3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n-=3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n = (y*image_width+bx)*3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n+=3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n+=3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
        }
        for (x=tx;x<=bx;x++) {
            n = (ty*image_width+x)*3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n-=stride;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n-=stride;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n = (by*image_width+x)*3;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n+=stride;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
            n+=stride;
            image_data[n]=r;
            image_data[n+1]=g;
            image_data[n+2]=b;
        }
    }
}

/* Shows the region of the image searched */
void trafficlights_show_search_region(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * lookup,
    int horizon,
    unsigned short * lanes)
{
    int x, y, n, y_bottom=0, y_top=0,found;
    int stride = image_width*3;

    if (lanes[0]==0) return;
    for (x = 0; x < image_width; x++) {
        found=0;
        if (geometry_vertical_range(
            image_width, image_height, x,
            lookup, horizon, lanes,
            &y_bottom, &y_top)!=0) {
            if (geometry_vertical_range(
                image_width, image_height, x,
                lookup, horizon, &(lanes[4]),
                &y_bottom, &y_top)==0) {
                found=2;
            }
        }
        else {
            found=1;
        }

        if (found!=0) {
            n = x*3;
            for (y=0;y<image_height;y++,n+=stride) {
                if ((y<y_top) || (y>y_bottom)) {
                    image_data[n] = 0;
                    image_data[n+1] = 0;
                    image_data[n+2] = 0;
                }
            }
        }
        else {
            n = x*3;
            for (y=0;y<image_height;y++,n+=stride) {
                image_data[n] = 0;
                image_data[n+1] = 0;
                image_data[n+2] = 0;
            }
        }
    }
}

/* Do the left, right and top sides of the given region have
   distinctive edges? */
int trafficlights_validate_edges(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    int threshold)
{
    int x,y,n,r,r2,edges=0;
    int region_width = bx-tx;
    int stride = image_width*3;
    int validated=0;
    int threshold_vertical, threshold_horizontal;

    r = (region_width>>2)*3;
    if ((tx > r) && (bx < image_width-r) &&
        (ty>8) && (by < image_height-9)) {
        r2 = ((region_width>>3)+1)*3;
        threshold_vertical = threshold*(by-ty)*2;
        n = (ty*image_width+tx)*3;
        for (y=ty;y<by;y++,n+=stride) {
            edges += (image_data[n-r]-image_data[n+r])+
                     (image_data[n-r2]-image_data[n+r2]);
        }
        if (edges>threshold_vertical) {
            edges=0;
            n = (ty*image_width+bx)*3;
            for (y=ty;y<by;y++,n+=stride) {
                edges += (image_data[n+r]-image_data[n-r])+
                         (image_data[n+r2]-image_data[n-r2]);
            }
            if (edges > threshold_vertical) {
                threshold_horizontal=threshold*(bx-tx)*2;
                edges=0;
                r = stride*4;
                r2 = stride*8;
                n = (ty*image_width+tx)*3;
                for (x=tx;x<bx;x++,n+=3) {
                    edges +=
                        (image_data[n-r]-image_data[n+r])+
                        (image_data[n-r2]-image_data[n+r2]);
                }
                if (edges>threshold_horizontal) {
                    edges=0;
                    n = (by*image_width+tx)*3;
                    for (x=tx;x<bx;x++,n+=3) {
                        edges +=
                            (image_data[n+r]-image_data[n-r])+
                            (image_data[n+r2]-image_data[n-r2]);
                    }
                    if (edges>threshold_horizontal) {
                        validated=1;
                    }
                }
            }
        }
    }

    return validated;
}

/* Removes thin vertically oriented lines.
   This improves the chances that subsequent flood fill will find
   separate rectangular areas */
void trafficlights_filter_thin_verticals(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    int minimum_width)
{
    int x,xx,y,n=0,n2,state,prev_x,colour_found;
    int stride = image_width*3;

    if (horizon > image_height-6) {
        horizon = image_height-6;
    }

    n = 5*image_width*3;
    for (y=5;y<horizon;y++) {
        state=0;
        prev_x=-1;
        for (x=0;x<image_width;x++,n+=3) {
            if (image_traffic_lights[n+layer]>TRAFFICLIGHT_COLOUR) {
                if (state==0) {
                    state=1;
                    prev_x=x;
                }
            }
            if ((state==1) &&
                (image_traffic_lights[n+layer]==0)) {
                state=0;
                if (x-prev_x < minimum_width) {
                    colour_found=0;
                    n2 = (y*image_width + prev_x)*3;
                    for (xx=prev_x;xx>prev_x-10;xx--,n2-=3) {
                        if (xx<0) break;
                        if ((image_traffic_lights[n2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+stride*4]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+stride*4+1]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-stride*4]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-stride*4+1]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+stride*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+stride*2+1]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-stride*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-stride*2+1]==
                                TRAFFICLIGHT_COLOUR)) {
                            colour_found=1;
                        }
                    }
                    if (colour_found==0) {
                        n2 = (y*image_width + x)*3;
                        for (xx=x; xx < x+10; xx++, n2+=3) {
                            if (xx >= image_width) break;
                            if ((image_traffic_lights[n2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2+stride*4]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2+stride*4+1]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2-stride*4]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2-stride*4+1]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2+stride*2]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2+stride*2+1]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2-stride*2]==TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[
                                    n2-stride*2+1]==TRAFFICLIGHT_COLOUR)) {
                                colour_found=1;
                            }
                        }
                        if (colour_found==0) {
                            n2 = (y*image_width + prev_x)*3+layer;
                            for (xx=prev_x;xx<x;xx++,n2+=3) {
                                image_traffic_lights[n2]=0;
                            }
                        }
                    }
                }
            }
        }
    }
}

/* Removes thin horizontally oriented lines.
   This improves the chances that subsequent flood fill will find
   separate rectangular areas */
void trafficlights_filter_thin_horizontals(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    int minimum_width)
{
    int x,yy,y,n=0,n2,state,prev_y,colour_found;
    int stride = image_width*3;

    if (horizon > image_height-6) {
        horizon = image_height-6;
    }

    n = 5*image_width*3;
    for (x=0;x<image_width;x++) {
        state=0;
        prev_y=-1;
        n = (5*image_width + x)*3;
        for (y=5;y<horizon;y++,n+=stride) {
            if (image_traffic_lights[n+layer]>TRAFFICLIGHT_COLOUR) {
                if (state==0) {
                    state=1;
                    prev_y=y;
                }
            }
            if ((state==1) &&
                (image_traffic_lights[n+layer]==0)) {
                state=0;
                if (y-prev_y < minimum_width) {
                    colour_found=0;
                    n2 = (prev_y*image_width + x)*3;
                    for (yy=prev_y;yy>prev_y-10;yy--,n2-=stride) {
                        if (yy<0) break;
                        if ((image_traffic_lights[n2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+3*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1+3*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-3*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1-3*2]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+3*4]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1+3*4]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2-3*4]==
                                TRAFFICLIGHT_COLOUR) ||
                            (image_traffic_lights[n2+1-3*4]==
                                TRAFFICLIGHT_COLOUR)) {
                            colour_found=1;
                        }
                    }
                    if (colour_found==0) {
                        n2 = (y*image_width + x)*3;
                        for (yy=y; yy < y+10; yy++, n2+=stride) {
                            if (yy >= image_height) break;
                            if ((image_traffic_lights[n2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+3*2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1+3*2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2-3*2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1-3*2]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+3*4]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1+3*4]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2-3*4]==
                                    TRAFFICLIGHT_COLOUR) ||
                                (image_traffic_lights[n2+1-3*4]==
                                    TRAFFICLIGHT_COLOUR)) {
                                colour_found=1;
                            }
                        }
                        if (colour_found==0) {
                            n2 = (prev_y*image_width + x)*3+layer;
                            for (yy=prev_y;yy<y;yy++,n2+=stride) {
                                image_traffic_lights[n2]=0;
                            }
                        }
                    }
                }
            }
        }
    }
}

/* given a reflectance threshold try to locate pixels which
   could belong to traffic lights which are above the threshold */
void trafficlights_filter_colours(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int * lookup,
    int horizon,
    int layer,
    unsigned short * lanes,
    int reflectance_threshold)
{
    int x,y,found,y_top=0,y_bottom=0,n,r,g,b,v;
    int stride = image_width*3;
    int max_green=0,max_red=0,hits;

    if (lanes[0]==0) return;

    /* find the maximum red and green responses */
    for (x = 0; x < image_width; x++) {
        found=0;
        if (geometry_vertical_range(
            image_width, image_height, x,
            lookup, horizon, lanes,
            &y_bottom, &y_top)!=0) {
            if (geometry_vertical_range(
                image_width, image_height, x,
                lookup, horizon, &(lanes[4]),
                &y_bottom, &y_top)==0) {
                found=2;
            }
        }
        else {
            found=1;
        }

        if (found!=0) {
            hits=0;
            n = (y_top*image_width+x)*3;
            for (y=y_top;y<y_bottom;y++,n+=stride) {
                if (image_traffic_lights[n]!=0) hits++;
            }
            if (hits>8) {
                n = (y_top*image_width+x)*3;
                for (y=y_top;y<y_bottom;y++,n+=stride) {
                    r = image_data[n];
                    g = image_data[n+1];
                    b = image_data[n+2];
                    /* above the threshold ? */
                    if (r+g+b > reflectance_threshold) {
                        /* store the maximum red and green responses */
                        if (g*2-r-b > max_green) {
                            max_green = g*2-r-b;
                        }
                        if (r*2-g-b > max_red) {
                            max_red = r*2-g-b;
                        }
                    }
                }
            }
        }
    }

    /* convert the maximum red and green responses
       into threshold values */
    if (max_green>TRAFFICLIGHTS_COLOUR_THRESHOLD) {
        max_green = MULDIVINT32(max_green,
            TRAFFICLIGHTS_GREEN_THRESHOLD,100);
    }
    else {
        max_green = 0;
    }
    if (max_red > TRAFFICLIGHTS_COLOUR_THRESHOLD) {
        max_red = MULDIVINT32(max_red,TRAFFICLIGHTS_RED_THRESHOLD,100);
    }
    else {
        max_red = 0;
    }

    /* apply the thresholds to the traffic lights image */
    if ((max_green>0) || (max_red>0)) {
        for (x = 0; x < image_width; x++) {
            found=0;
            if (geometry_vertical_range(
                image_width, image_height, x,
                lookup, horizon, lanes,
                &y_bottom, &y_top)!=0) {
                if (geometry_vertical_range(
                    image_width, image_height, x,
                    lookup, horizon, &(lanes[4]),
                    &y_bottom, &y_top)==0) {
                    found=2;
                }
            }
            else {
                found=1;
            }

            if (found!=0) {
                hits=0;
                n = (y_top*image_width+x)*3;
                for (y=y_top;y<y_bottom;y++,n+=stride) {
                    if (image_traffic_lights[n]!=0) hits++;
                }
                if (hits>8) {
                    n = (y_top*image_width+x)*3;
                    for (y=y_top;y<y_bottom;y++,n+=stride) {
                        r = image_data[n];
                        g = image_data[n+1];
                        b = image_data[n+2];
                        if (r+g+b > reflectance_threshold) {
                            v = g*2-r-b;
                            if ((max_green>0) && (v>max_green)) {
                                image_traffic_lights[n]=0;
                                image_traffic_lights[n+1]=
                                    TRAFFICLIGHT_COLOUR;
                                image_traffic_lights[n+2]=0;
                            }
                            else {
                                v = r*2-g-b;
                                if ((max_red>0) && (v>max_red)) {
                                    image_traffic_lights[n]=
                                        TRAFFICLIGHT_COLOUR;
                                    image_traffic_lights[n+1]=0;
                                    image_traffic_lights[n+2]=0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


/* Identifies dark pixels which could be traffic lights */
void trafficlights_filter(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int * lookup,
    int horizon,
    int layer,
    unsigned short * lanes,
    int reflectance_threshold,
    int variance_threshold)
{
    int reflectance;
    int x,y,found,y_top=0,y_bottom=0,n,r,g,b,variance;
    int stride = image_width*3;

    if (lanes[0]==0) return;

    reflectance = trafficlights_max_reflectance(
        image_width, image_height, image_data,
        lookup, horizon, lanes,
        reflectance_threshold,
        variance_threshold);
    if (reflectance > 0) {
        reflectance*=3;
        reflectance = MULDIVINT32(reflectance,110,100);

        for (x = 10; x < image_width-10; x++) {
            found=0;
            if (geometry_vertical_range(
                image_width, image_height, x,
                lookup, horizon, lanes,
                &y_bottom, &y_top)!=0) {
                if (geometry_vertical_range(
                    image_width, image_height, x,
                    lookup, horizon, &(lanes[4]),
                    &y_bottom, &y_top)==0) {
                    found=2;
                }
            }
            else {
                found=1;
            }

            if (found!=0) {
                n = (y_top*image_width+x)*3;
                for (y=y_top;y<y_bottom;y++,n+=stride) {
                    r = image_data[n];
                    g = image_data[n+1];
                    b = image_data[n+2];
                    variance = ABS(r-b)+ABS(r-g)+ABS(g-b);
                    if ((r+g+b < reflectance) &&
                        (variance < variance_threshold)) {
                        image_traffic_lights[n+layer]=255;
                    }
                }
            }
        }

        trafficlights_filter_colours(
            image_width, image_height,
            image_data, image_traffic_lights,
            lookup, horizon, layer,
            lanes, reflectance);

        trafficlights_filter_thin_verticals(
            image_width, image_height,
            image_traffic_lights,
            horizon, layer, 5);
        trafficlights_filter_thin_horizontals(
            image_width, image_height,
            image_traffic_lights,
            horizon, layer, 5);
    }
}

/* Finds the top and bottom of the given region.
   This ensures that any thin areas such as the pole upon which
   the traffic light is perched are removed from further consideration */
void trafficlights_update_region(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int layer,
    int * tx,
    int * ty,
    int * bx,
    int * by)
{
    int x,y,n,n2,w,left,right,top=-1,bottom=-1;
    int cx = *tx+((*bx-*tx)>>1);
    int stride = image_width*3;

    w = (*bx-*tx)>>1;
    n = (*ty*image_width+*tx)*3+layer;
    for (y=*ty;y<*by;y++,n+=stride) {
        n2=n;
        for (x=*tx;x<cx;x++,n2+=3) {
            if (image_traffic_lights[n2]!=0) break;
        }
        left = x-*tx;
        n2=(y*image_width+*bx)*3+layer;
        for (x=*bx;x>cx;x--,n2-=3) {
            if (image_traffic_lights[n2]!=0) break;
        }
        right = *bx-x;
        if (left+right<w) {
            if (top==-1) {
                top = y;
            }
            bottom = y;
        }
    }
    *ty = top;
    *by = bottom;
}

int trafficlights_validate_edge_variance(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int layer,
    int tx,
    int ty,
    int bx,
    int by,
    int threshold)
{
    int x,y,n,n2;
    int cx = tx+((bx-tx)>>1);
    int stride = image_width*3;
    int variance=0, validated=0;

    threshold = MULDIVINT32(threshold*(by-ty),image_width,320);

    n = (ty*image_width+tx)*3+layer;
    for (y=ty;y<by;y++,n+=stride) {
        n2=n;
        for (x=tx;x<cx;x++,n2+=3) {
            if (image_traffic_lights[n2]!=0) break;
        }
        variance += (x-tx)<<2;
    }
    if (variance < threshold) {
        variance=0;
        n = (ty*image_width+bx)*3+layer;
        for (y=ty;y<by;y++,n+=stride) {
            n2=n;
            for (x=bx;x>cx;x--,n2-=3) {
                if (image_traffic_lights[n2]!=0) break;
            }
            variance += (bx-x)<<2;
        }
        if (variance < threshold) {
            validated=1;
        }
    }
    return validated;
}


/* Does the given region have appropriate properties? */
int trafficlights_validate(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned short * lanes,
    int * colour,
    int layer,
    int edge_threshold,
    int edge_variance_threshold)
{
    int aspect,validated=0;
    if (bx-tx<3) return 0;
    if ((tx<5) || (tx>image_width-5) ||
        (ty<5) || (ty>image_height-5)) {
        return 0;
    }

    aspect = MULDIVINT32((by-ty),100,(bx-tx));
    if (aspect < TRAFFICLIGHTS_ASPECT_MIN) {
        return 0;
    }
    if (aspect > TRAFFICLIGHTS_ASPECT_MAX) {
        return 0;
    }
    *colour = trafficlights_colour(
        image_width, image_height,
        image_traffic_lights,
        tx, ty, bx, by);
    if (*colour>-1) {
        if (trafficlights_validate_edges(
                image_width, image_height, image_data,
                tx, ty, bx, by, edge_threshold)!=0) {
            validated =
                trafficlights_validate_edge_variance(
                    image_width, image_height,
                    image_traffic_lights,
                    layer, tx, ty, bx, by,
                    edge_variance_threshold);
        }
    }

    return validated;
}

/* Returns the colour of a traffic light, or -1 if
   no colour was detected */
int trafficlights_colour(
    int image_width,
    int image_height,
    unsigned char * image_traffic_lights,
    int tx,
    int ty,
    int bx,
    int by)
{
    int x,y,n;
    int red_y=0,red_hits=0;
    int green_y=0,green_hits=0;
    int colour=-1;

    for (y=ty;y<=by;y++) {
        n = (y*image_width+tx)*3;
        for (x=tx;x<=bx;x++,n+=3) {
            if (image_traffic_lights[n+1]==TRAFFICLIGHT_COLOUR) {
                green_y += y;
                green_hits++;
            }
            else {
                if (image_traffic_lights[n]==TRAFFICLIGHT_COLOUR) {
                    red_y += y;
                    red_hits++;
                }
            }
        }
    }

    if ((red_hits>2) || (green_hits>2)) {
        if (red_hits>green_hits) {
            red_y/=red_hits;
            if (red_y < ty+MULDIVINT32((by-ty),60,100)) {
                colour = TRAFFICLIGHT_STATE_RED;
            }
            else {
                if (red_y > ty+MULDIVINT32((by-ty),30,100)) {
                    colour = TRAFFICLIGHT_STATE_AMBER;
                }
            }
        }
        else {
            if (green_hits>0) {
                green_y/=green_hits;
                if (green_y > ty+MULDIVINT32((by-ty),60,100)) {
                    colour = TRAFFICLIGHT_STATE_GREEN;
                }
            }
        }
    }
    return colour;
}

/* Detect traffic lights in the given filtered image */
int trafficlights_detect(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_traffic_lights,
    int horizon,
    int layer,
    unsigned short * stack,
    unsigned short * lanes,
    unsigned short * regions)
{
    int tx=0,ty=0,bx=0,by=0,x,y,n=0;
    int colour=-1,no_of_regions=0,aspect;
    int stack_size = (image_width>>2)*(image_height>>2)*2;

    for (y=0;y<horizon;y++) {
        for (x=0;x<image_width;x++,n+=3) {
            if (image_traffic_lights[n+layer]==255) {
                floodfill_fill_layer(
                    x, y, image_width, image_height,
                    stack, stack_size,
                    image_traffic_lights,
                    layer, 255, 254,
                    &tx,&ty,&bx,&by);

                if (bx>tx) {
                    aspect = MULDIVINT32((by-ty),100,(bx-tx));
                    if ((aspect >= TRAFFICLIGHTS_ASPECT_MIN) &&
                        (aspect <= TRAFFICLIGHTS_ASPECT_MAX2)) {
                        trafficlights_update_region(
                            image_width,
                            image_height,
                            image_traffic_lights, layer,
                            &tx,&ty,&bx,&by);
                        if (trafficlights_validate(
                            image_width, image_height,
                            image_data,
                            image_traffic_lights,
                            tx,ty,bx,by,lanes,&colour,layer,
                            TRAFFICLIGHTS_EDGES_THRESHOLD,
                            TRAFFICLIGHTS_EDGES_VARIANCE_THRESHOLD)!=0) {
                            regions[no_of_regions*
                                TRAFFICLIGHTS_FIELDS+
                                TRAFFICLIGHTS_FIELD_TX]=tx;
                            regions[no_of_regions*
                                TRAFFICLIGHTS_FIELDS+
                                TRAFFICLIGHTS_FIELD_TY]=ty;
                            regions[no_of_regions*
                                TRAFFICLIGHTS_FIELDS+
                                TRAFFICLIGHTS_FIELD_BX]=bx;
                            regions[no_of_regions*
                                TRAFFICLIGHTS_FIELDS+
                                TRAFFICLIGHTS_FIELD_BY]=by;
                            regions[no_of_regions*
                                TRAFFICLIGHTS_FIELDS+
                                TRAFFICLIGHTS_FIELD_COLOUR]=colour;
                            no_of_regions++;
                            if (no_of_regions==TRAFFICLIGHTS_MAX) {
                                y=horizon;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return no_of_regions;
}

/* Returns the current traffic light state */
int trafficlights_online_update(
    int lights_state,
    unsigned short * traffic_light_history,
    int minimum_observations)
{
    int i,index=-1,state;
    int min = TRAFFICLIGHTS_HISTORY_COUNTER_MAX+1;
    int max = minimum_observations;
    int current_state = TRAFFICLIGHT_STATE_UNKNOWN;
    unsigned char votes[4];

    memset((void*)votes,'\0',4);
    for (i=0;i<TRAFFICLIGHTS_HISTORY*TRAFFICLIGHTS_HISTORY_FIELDS;
         i+=TRAFFICLIGHTS_HISTORY_FIELDS) {
        /* decrement counters */
        if (traffic_light_history[
            i+TRAFFICLIGHTS_HISTORY_FIELD_COUNTER] > 0) {
            traffic_light_history[i]--;
        }
        else {
            /* if the counter is zero then set the state to unknown */
            traffic_light_history[
                i+TRAFFICLIGHTS_HISTORY_FIELD_STATE] =
                    TRAFFICLIGHT_STATE_UNKNOWN;
        }
        /* does this entry have the lowest counter value? */
        if (traffic_light_history[
                i+TRAFFICLIGHTS_HISTORY_FIELD_COUNTER] < min) {
            index=i;
            min = traffic_light_history[
                i+TRAFFICLIGHTS_HISTORY_FIELD_COUNTER];
        }
        /* update state votes */
        state = traffic_light_history[i+TRAFFICLIGHTS_HISTORY_FIELD_STATE];
        if (state != TRAFFICLIGHT_STATE_UNKNOWN) {
            votes[state]++;
        }
    }
    /* add the new data */
    if ((index > -1) && (lights_state != TRAFFICLIGHT_STATE_UNKNOWN)) {
        traffic_light_history[
            index+TRAFFICLIGHTS_HISTORY_FIELD_COUNTER] =
                TRAFFICLIGHTS_HISTORY_COUNTER_MAX;
        traffic_light_history[
            index+TRAFFICLIGHTS_HISTORY_FIELD_STATE] = lights_state;
        votes[lights_state]++;
    }
    /* which state has the most votes? */
    for (index = 0; index < 4; index++) {
        if (votes[index] > max) {
            max = votes[index];
            current_state = index;
        }
    }

    return current_state;
}

