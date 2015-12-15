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

#include "roadmarkings.h"

/* draws a line in an image */
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
    int line_only)
{
    int x,y,n,y0,y1,dx,dy;
    if (line_only!=0) {
        y0 = ty;
        y1 = by;
    }
    else {
        y0 = 0;
        y1 = image_height;
    }
    dx = bx-tx;
    dy = by-ty;

    if (dy != 0) {
        for (y=y0;y<y1;y++) {
            x = tx + MULDIVINT32(y-ty,dx,dy);
            if ((x>0) && (x<image_width)) {
                n = (y*image_width+x)*3;
                image_data[n]=r;
                image_data[n+1]=g;
                image_data[n+2]=b;
            }
        }
    }
}

/* Is a line on the ground plane equivalent to a vertical line
   in the original image?  We can assume that vertical lines
   in the original image are probably not road markings */
int roadmarkings_ground_plane_line_is_vertical(
    int ground_plane_tx,
    int ground_plane_ty,
    int ground_plane_bx,
    int ground_plane_by,
    int image_width,
    int projected_width,
    int * projected_lookup)
{
    int is_vertical=0;
    int n, x1, x2, y;
    int stride = image_width*3;

    n = projected_lookup[ground_plane_ty*projected_width+ground_plane_tx];
    y = n/stride;
    x1 = (n/3) - y*image_width;
    n = projected_lookup[ground_plane_by*projected_width+ground_plane_bx];
    y = n/stride;
    x2 = (n/3) - y*image_width;
    if (ABS(x1 - x2) < 5) {
        is_vertical=1;
    }

    return is_vertical;
}

/* Fits a line to an ordered set of points.
   This isn't RANSAC, but the principle is similar. */
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
    int * projected_lookup)
{
    int i,j,k,dx,dy,dx2,x,offset,tx,ty;
    int stride,hits,max_hits=0;
    int min_baseline = MULDIVINT32(no_of_points,5,100);
    int max_baseline = no_of_points/2;
    if (min_baseline<4) min_baseline=4;
    stride = projected_width*3;
    for (offset = min_baseline;offset<max_baseline; offset++) {
        for (i=0;i<no_of_points-offset;i++) {
            tx = points[i*2];
            ty = points[i*2 + 1];
            j = i+offset;
            dx = points[j*2]- tx;
            dy = points[j*2+1]- ty;
            if (dy!=0) {
                x = tx + MULDIVINT32(((projected_height-1) - ty),dx,dy);
                if ((x>min_start_x) && (x<max_start_x)) {

                    if (roadmarkings_ground_plane_line_is_vertical(
                        points[i*2], points[i*2+1],
                        points[j*2], points[j*2+1],
                        image_width, projected_width,
                        projected_lookup)==0) {

                        hits=0;
                        for (k=0;k<no_of_points;k++) {
                            x = tx +
                                MULDIVINT32((points[k*2+1] - ty),dx,dy);
                            dx2 = ABS(x-points[k*2]);
                            if (dx2 < 10) {
                                hits += (10-dx2)*(10-dx2);
                            }
                        }
                        if (hits>max_hits) {
                            max_hits=hits;
                            curb[0] = points[i*2];
                            curb[1] = points[i*2+1];
                            curb[2] = points[j*2];
                            curb[3] = points[j*2+1];
                        }
                    }
                }
            }
        }
    }
    return max_hits;
}

/* reprojects a line from the ground plane back into
   the original image coordinates */
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
    int line_range_mm)
{
    int tx,ty,bx,by,min_y,y,dx,dy,n,x0,x1=0;

    min_y =
        projected_height-1-
        ((line_range_mm - ground_plane_min_range_mm)*
        projected_height/
        (ground_plane_max_range_mm-ground_plane_min_range_mm));

    dx = (int)(line[2] - line[0]);
    dy = (int)(line[3] - line[1]);
    if (dy!=0) {
        x0 = (int)line[0] + MULDIVINT32((min_y - (int)line[1]),dx,dy);
        n = projected_lookup[min_y*projected_width+x0]/3;
        ty = n/image_width;
        tx = n - ty*image_width;
        y = projected_height-1;
        n=0;
        while ((n==0) && (y>0)) {
            x1 = (int)line[0] + MULDIVINT32((y - (int)line[1]),dx,dy);
            n = projected_lookup[y*projected_width+x1]/3;
            y--;
        }
        by = n/image_width;
        bx = n - by*image_width;
        line_projected[0] = (unsigned short)tx;
        line_projected[1] = (unsigned short)ty;
        line_projected[2] = (unsigned short)bx;
        line_projected[3] = (unsigned short)by;

        line[0] = x1;
        line[1] = y;
        line[2] = x0;
        line[3] = min_y;
    }
}

/* reprojects a pair of lines from the ground plane back
   into the original image coordinates */
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
    int line_range_mm)
{
    roadmarkings_reproject_line(
        image_width, image_height, image_data,
        projected_width, projected_height,
        ground_plane_min_range_mm,
        ground_plane_max_range_mm,
        &(lanes[0]), &(lanes[8]),
        projected_lookup,
        line_range_mm);

    roadmarkings_reproject_line(
        image_width, image_height, image_data,
        projected_width, projected_height,
        ground_plane_min_range_mm,
        ground_plane_max_range_mm,
        &(lanes[4]), &(lanes[12]),
        projected_lookup,
        line_range_mm);
}

/* draws detected lanes within the original image */
void roadmarkings_draw_lanes(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned short * lanes_projected,
    int r, int g, int b,
    int line_width)
{
    int lane,tx,ty,bx,by,offset;

    for (lane=0;lane<2;lane++) {
        tx = lanes_projected[lane*4];
        ty = lanes_projected[lane*4+1];
        bx = lanes_projected[lane*4+2];
        by = lanes_projected[lane*4+3];
        for (offset=-line_width;offset<=line_width;offset++) {
            draw_line(
                image_width, image_height, image_data,
                tx+offset, ty, bx+offset, by, r,g,b,1);
        }
    }
}

/* apply a sobel edge filter to the given ground plane image */
void roadmarkings_edges(
    int projected_width,
    int projected_height,
    unsigned char * ground_plane,
    unsigned char * ground_plane_edges)
{
    int i,gx,gy,magnitude,start;
    int stride = projected_width*3;

    /* clear the edges image */
    memset(
        (void*)ground_plane_edges,'\0',
        projected_width*projected_height*3);

    /* start half way down the image, since we're not interested in edges
       at very long range which are likely to be inaccurate */
    start = projected_width*(projected_height/2)*3 + 3;

    /* for every pixel in the lower half of the image */
    for (i = start;
         i < projected_width*projected_height*3 - (stride-3);
         i+=3) {
        gx = ground_plane[i-stride-3]*-1;
        gx += ground_plane[i-3]*-2;
        gx += ground_plane[i+stride-3]*-1;
        gx += ground_plane[i-stride+3];
        gx += ground_plane[i+3]*2;
        gx += ground_plane[i+stride+3];

        gy = ground_plane[i-stride-3];
        gy += ground_plane[i-stride]*2;
        gy += ground_plane[i-stride+3];
        gy += ground_plane[i+stride-3]*-1;
        gy += ground_plane[i+stride]*-2;
        gy += ground_plane[i+stride+3]*-1;

        magnitude = (ABS(gx)+ABS(gy))>>1;
        ground_plane_edges[i] = magnitude;
    }
}


/* detect road markings */
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
    int * projected_lookup)
{
    int left_x,right_x,y,i,n0,n1;
    int edge_magnitude, start_x, end_x,expansion;
    int best_left=0, best_right=0, av_lane_width=0, hits=0;
    int max_score,score,min_lane_width, max_lane_width,lane_width;
    int w0 = MULDIVINT32(projected_width,20,100);
    int w1 = MULDIVINT32(projected_width,80,100);
    int stride0,stride1,stride2,stride3;
    int max_x, no_of_points_left=0;
    unsigned short points_left[256];
    int no_of_points_right=0;
    unsigned short points_right[256];

    /* clear the lane markings image */
    memset((void*)markings,'\0',projected_width*projected_height*3);

    /* compute an edges image by applying a sobel filter to the
       ground plane image */
    roadmarkings_edges(
        projected_width, projected_height, ground_plane, markings);

    /* calculate the average lane width from the
       segmented ground plane image */
    n0=0;
    for (y=0;y<projected_height;y++) {
        i=0;
        for (left_x=0;left_x<projected_width;left_x++,n0+=3) {
            if (segmented[n0+1]==255) {
                i++;
            }
        }
        if (i>4) {
            av_lane_width += i;
            hits++;
        }
    }
    if (hits>0) {
        av_lane_width /= hits;
    }

    /* min and max lane widths in ground plane coordinates */
    min_lane_width =
        MULDIVINT32(MULDIVINT32(vehicle_width_mm,120,100),
        projected_width, ground_plane_width_mm);
    max_lane_width =
        MULDIVINT32(MULDIVINT32(vehicle_width_mm,250,100),
        projected_width, ground_plane_width_mm);

    /* Reduce the lane width if the average indicates
       that the width is narrow.
       This only usually applies to narrow tracks or driveways */
    if ((av_lane_width > 0) && (min_lane_width > av_lane_width)) {
        min_lane_width = MULDIVINT32(av_lane_width,80,100);
    }

    /* What is the minimum x coordinate in the ground plane image (w0)
       which appears on the far left of the raw image? */
    w0 = 0;
    left_x = 0;
    while ((w0==0) && (left_x<projected_width/2)) {
        i = projected_lookup[(projected_height-1)*projected_width + left_x];
        if (i>0) w0 = left_x;
        left_x++;
    }
    w0 -= MULDIVINT32(projected_width,10,100);
    /* the maximum x coordinate (w1) we assume to be the inverse */
    w1 = (projected_width/2) + ((projected_width/2) - w0);

    /* This governs the rate of expansion in the horizontal
       search for the road.  Note that it's not necessarily connected
       to field of view, but instead has more to do with the typical
       rate of bend found in most roads */
    expansion = projected_width;

    max_x = MULDIVINT32(projected_width,45,100);

    /* pre-compute various stride values to save time later */
    stride0 = projected_width*3*2;
    stride1 = projected_width*3*4;
    stride2 = projected_width*3*6;
    stride3 = projected_width*3*8;

    /* from the bottom to half way up the ground plane image
       (longer distances are likely to be significantly less accurate) */
    for (y = projected_height-2;
         y > MULDIVINT32(projected_height,50,100);
         y--) {
        max_score=0;
        /* start x coordinate for possible left lane marking */
        start_x =
            w0 -
            MULDIVINT32((projected_height-y),expansion,projected_height);
        if (start_x<0) start_x=0;
        /* for each possible lane width */
        for (lane_width = min_lane_width;
             lane_width < max_lane_width;
             lane_width++) {
            /* end x coordinate for possible left lane marking */
            end_x =
                w1 +
                MULDIVINT32((projected_height-y),
                expansion,projected_height) - lane_width;
            if (end_x > max_x) {
                end_x=max_x;
            }
            /* for each possible horizontal offset */
            for (left_x = start_x;left_x < end_x;left_x++) {
                n0 = (y*projected_width+left_x)*3;
                /* does segmentation indicate this
                   pixel belongs to the road? */
                if ((segmented[n0+1]==255) || (segmented[n0]==255)) {
                    /* position of the possible right lane marking */
                    right_x = left_x + lane_width;
                    n1 = (y*projected_width+right_x)*3;
                    /* does segmentation indicate this pixel
                       belongs to the road? */
                    if ((segmented[n1+1]==255) || (segmented[n1]==255)) {
                        /* edge value for the possible
                           left lane marking */
                        edge_magnitude=0;
                        for (i = n0-3; i < n0; i+=3) {
                            edge_magnitude += markings[i]+
                                markings[i-stride0]+
                                markings[i-stride1]+
                                markings[i-stride2]+
                                markings[i-stride3];
                        }
                        for (i=n0; i <= n0+3; i+=3) {
                            edge_magnitude -= markings[i]+
                                markings[i-stride0]+
                                markings[i-stride1]+
                                markings[i-stride2]+
                                markings[i-stride3];
                        }
                        /* is this a significant edge value? */
                        if (ABS(edge_magnitude) > edge_threshold) {
                            score = ABS(edge_magnitude);
                            /* edge value for the possible
                               right lane marking */
                            edge_magnitude=0;
                            for (i = n1-3; i < n1; i += 3) {
                                edge_magnitude += markings[i]+
                                    markings[i-stride0]+
                                    markings[i-stride1]+
                                    markings[i-stride2]+
                                    markings[i-stride3];
                            }
                            for (i = n1; i <= n1+3; i += 3) {
                                edge_magnitude -= markings[i]+
                                    markings[i-stride0]+
                                    markings[i-stride1]+
                                    markings[i-stride2]+
                                    markings[i-stride3];
                            }
                            /* is this a significant edge value? */
                            if (ABS(edge_magnitude) > edge_threshold) {
                                score += ABS(edge_magnitude);
                                /* is the total edge response maximal? */
                                if (score > max_score) {
                                    max_score = score;
                                    /* keep note of the best possible
                                       left and right lane marking
                                       x coordinates */
                                    best_left = left_x;
                                    best_right = right_x;
                                }
                            }
                        }
                    }
                }
            }
        }
        /* do we have a winner? */
        if (max_score > edge_threshold) {
            if (no_of_points_left<128) {
                /* store the positions of the left and right
                   sides of the lane.  Note that these coordinates
                   are for the projected image, not the original */
                points_left[no_of_points_left*2] = best_left;
                points_left[no_of_points_left*2+1] = y;
                points_right[no_of_points_right*2] = best_right;
                points_right[no_of_points_right*2+1] = y;
                no_of_points_left++;
                no_of_points_right++;
            }
            else break;
            /* draw the detections for visualisation/debugging */
            i = (y*projected_width+best_left)*3;
            markings[i+1]=255;
            segmented[i]=255;
            segmented[i+1]=0;
            segmented[i+2]=0;
            i = (y*projected_width+best_right)*3;
            markings[i+1]=255;
            segmented[i]=255;
            segmented[i+1]=0;
            segmented[i+2]=0;
            hits++;
        }
    }
    if (hits>0) {
        /* detect the left side of the lane from the points */
        hits = roadmarkings_line_fit(
            no_of_points_left,
            points_left,
            w0, projected_width/2,
            projected_width, projected_height, segmented, lanes,
            image_width, projected_lookup);

        if (hits > line_threshold) {
            draw_line(
                projected_width, projected_height,
                markings,
                lanes[0], lanes[1], lanes[2], lanes[3], 255,0,0, 0);
            /* detect the right side of the lane from the points */
            hits = roadmarkings_line_fit(
                no_of_points_right,
                points_right,
                projected_width/2, w1,
                projected_width, projected_height,
                segmented, &lanes[4],
                image_width, projected_lookup);
            if (hits > line_threshold) {
                draw_line(
                    projected_width, projected_height, markings,
                    lanes[4], lanes[5], lanes[6], lanes[7],
                    255,0,0, 0);
            }
            else {
                /* lines don't look like a lane */
                for (i=4;i<8;i++) lanes[i]=0;
            }
        }
        else {
            /* lines don't look like a lane */
            for (i=0;i<4;i++) lanes[i]=0;
        }

        return av_lane_width;
    }
    else {
        return 0;
    }
}

