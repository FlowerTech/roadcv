/*
 vehicle related geometry
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

#include "vehiclegeometry.h"

/* creates a lookup table which for a given y axis coordinate
   returns the minimum and maximum y coordinates and width in pixels
   of an object above the ground plane */
void geometry_create_height_lookup(
    int image_width,
    int image_height,
    int * lookup,
    int min_height_mm,
    int max_height_mm,
    int width_mm,
    int horizon,
    int camera_height_mm,
    int fov,
    int tilt)
{
    int y, distance_mm, index=0;
    for (y = image_height-1; y > horizon; y--,index += 3) {
        distance_mm = geometry_pixel_distance(
            y, camera_height_mm, fov,
            image_width, image_height,
            tilt);

        lookup[index] = geometry_distance_forward_pixel(
            distance_mm, camera_height_mm, min_height_mm,
            fov, image_width, image_height, tilt);
        lookup[index+1] = geometry_distance_forward_pixel(
            distance_mm, camera_height_mm, max_height_mm,
            fov, image_width, image_height, tilt);
        lookup[index+2] = geometry_distance_right_pixel(
            distance_mm, width_mm, fov, image_width);
    }
}

/* Creates a lookup table which translates a y image coordinate
   into a range in millimetres, and also provides the minimum
   and maximum width in pixels for a vehicle at that range */
void geometry_create_range_lookup(
    int image_width,
    int image_height,
    int * lookup,
    int horizon,
    int camera_height_mm,
    int fov,
    int tilt,
    int min_width_mm,
    int max_width_mm)
{
    int y, index=0, distance_mm;
    int cx = image_width/2;
    for (y = image_height-1; y > horizon; y--,index+=3) {
        distance_mm = geometry_pixel_distance(
            y, camera_height_mm, fov,
            image_width, image_height,
            tilt);
        lookup[index] = distance_mm;
        lookup[index+1] = (geometry_distance_right_pixel(
            distance_mm, min_width_mm, fov, image_width)-cx);
        lookup[index+2] = (geometry_distance_right_pixel(
            distance_mm, max_width_mm, fov, image_width)-cx);
    }
}

int geometry_vertical_range(
    int image_width,
    int image_height,
    int x,
    int * lookup,
    int horizon,
    unsigned short * lanes,
    int * y_bottom,
    int * y_top)
{
    int y, dx, dy, dy_bottom, dy_top, index;
    int above_horizon=0;
    int max = image_height-horizon;
    if (lanes[0]==0) return 1;

    dy_bottom = lookup[0] - lookup[(max-1)*3];
    dy_top = lookup[1] - lookup[(max-1)*3+1];

    *y_bottom = 0;
    *y_top = 0;

    dx = lanes[2]-lanes[0];
    if (dx!=0) {
        dy = lanes[3] - lanes[1];
        y = lanes[1] + (x - lanes[0])*dy/dx;
        if ((y<image_height) && (y > horizon)) {
            index = image_height-1-y;

            *y_bottom = lookup[index*3];
            *y_top = lookup[index*3+1];
        }
        else {
            above_horizon = 1;
        }
    }
    return above_horizon;
}

int geometry_pixel_distance(
    int y,
    int camera_height_mm,
    int fov,
    int image_width,
    int image_height,
    int tilt)
{
    double tilt_radians = (double)tilt*3.1415927/180.0;
    double fov_radians =
        ((double)fov*3.1415927/180.0)*image_height/image_width;
    tilt_radians +=
        (y-(image_height*0.5))*(fov_radians*0.5)/(image_height*0.5);
    if (tilt_radians>0)
        return (int)(camera_height_mm/tan(tilt_radians));
    else
        return 1000000;
}

int geometry_pixel_right(
    int x,
    int distance_mm,
    int fov,
    int image_width)
{
    double fov_radians = (double)fov*3.1415927/180.0;
    double angle = (x*fov_radians/image_width)-(fov_radians*0.5);
    return (int)(distance_mm*tan(angle));
}

int geometry_distance_forward_pixel(
    int distance_mm,
    int camera_height_mm,
    int observed_point_height_mm,
    int fov,
    int image_width,
    int image_height,
    int tilt)
{
    double tilt_radians = (double)tilt*3.1415927/180.0;
    double fov_radians =
        ((double)fov*3.1415927/180.0)*image_height/image_width;

    double angle_radians =
        atan((camera_height_mm-observed_point_height_mm)/
             (double)distance_mm) - tilt_radians;
    int y =
        (image_height/2) + (int)(angle_radians*image_height/fov_radians);
    if (y < 0) y=0;
    if (y >= image_height) y = image_height-1;
    return y;
}

int geometry_distance_right_pixel(
    int distance_mm,
    int baseline_mm,
    int fov,
    int image_width)
{
    int x;
    double angle,fov_radians = (double)fov*3.1415927/180.0;

    angle = atan((double)baseline_mm/(double)distance_mm);
    x = (image_width/2) +
        (int)(angle * (image_width*0.5) / (fov_radians*0.5));
    return x;
}

void geometry_show_distances(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int camera_height_mm,
    int fov,
    int tilt,
    int min_distance_mm,
    int max_distance_mm,
    int distance_increment_mm,
    int camera_index,
    int baseline_mm,
    int road_width_mm,
    int offset_from_road_centre_mm)
{
    int dist,x,y,n,x1,y1,dx,dy,xx,yy,dist_left,dist_right;
    for (dist = min_distance_mm;
         dist < max_distance_mm;
         dist += distance_increment_mm) {
        y = geometry_distance_forward_pixel(
            dist,camera_height_mm,0,fov,image_width,image_height,tilt);
        n=y*image_width*3;
        for (x=0;x<image_width;x++,n+=3) {
            image_data[n]=0;
            image_data[n+1] = 0;
            image_data[n+2]=255;
        }
    }

    if (camera_index==0) {
        dist_left =
            (-road_width_mm/2)-(baseline_mm/2)+offset_from_road_centre_mm;
        dist_right =
            (road_width_mm/2)+(baseline_mm/2)+offset_from_road_centre_mm;
    }
    else {
        dist_left =
            (-road_width_mm/2)-(baseline_mm/2)+offset_from_road_centre_mm;
        dist_right =
            (road_width_mm/2)-(baseline_mm/2)+offset_from_road_centre_mm;
    }

    x = geometry_distance_right_pixel(
        max_distance_mm,dist_left,fov,image_width);
    y = geometry_distance_forward_pixel(
        max_distance_mm,camera_height_mm,0,
        fov,image_width,image_height,tilt);
    x1 = geometry_distance_right_pixel(
        min_distance_mm,dist_left,fov,image_width);
    y1 = geometry_distance_forward_pixel(
        min_distance_mm,camera_height_mm,0,
        fov,image_width,image_height,tilt);
    dx = x1-x;
    dy = y1-y;

    for (yy=y;yy<y1;yy++) {
        xx = x+((yy-y)*dx/(y1-y));
        n = (yy*image_width+xx)*3;
        image_data[n] = 0;
        image_data[n+1] = 0;
        image_data[n+2] = 255;
    }

    x = geometry_distance_right_pixel(
        max_distance_mm,dist_right,fov,image_width);
    y = geometry_distance_forward_pixel(
        max_distance_mm,camera_height_mm,0,
        fov,image_width,image_height,tilt);
    x1 = geometry_distance_right_pixel(
        min_distance_mm,dist_right,fov,image_width);
    y1 = geometry_distance_forward_pixel(
        min_distance_mm,camera_height_mm,0,
        fov,image_width,image_height,tilt);
    dx = x1-x;
    dy = y1-y;

    for (yy=y;yy<y1;yy++) {
        xx = x+((yy-y)*dx/(y1-y));
        if ((xx>0) && (xx<image_width)) {
            n = (yy*image_width+xx)*3;
            image_data[n] = 0;
            image_data[n+1] = 0;
            image_data[n+2] = 255;
        }
    }

}

