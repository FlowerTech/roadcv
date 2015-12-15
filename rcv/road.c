/*
 road location/segmentation
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

#include "road.h"

/* create lookup tables used to convert between the original
   image and ground plane projection */
void road_create_lookups(
    int image_width,
    int image_height,
    int min_range_mm,
    int max_range_mm,
    int camera_height_mm,
    int fov,
    int tilt,
    int vehicle_width_mm,
    int projected_width,
    int projected_height,
    int * projected_lookup,
    int * reprojected_lookup)
{
    int ix,iy,x,y,distance_x_mm,distance_y_mm,n;
    int i,j,n1,n2=0,n3;
    int prev_n2,prev_y=0,curr_y;

    /* clear the lookup tables */
    memset(
        (void*)projected_lookup,
        '\0',projected_width*projected_height*sizeof(int));
    memset(
        (void*)reprojected_lookup,
        '\0',image_width*image_height*sizeof(int));

    for (iy=0;iy<projected_height;iy++) {
        distance_y_mm =
            min_range_mm +
            (iy*(max_range_mm-min_range_mm)/projected_height);

        y = geometry_distance_forward_pixel(
            distance_y_mm,
            camera_height_mm,0,
            fov,
            image_width,
            image_height,
            tilt);

        for (ix=0;ix<projected_width;ix++) {
            distance_x_mm =
                (-vehicle_width_mm*3) +
                (ix*vehicle_width_mm*6/projected_width);

            x = geometry_distance_right_pixel(
                distance_y_mm,
                distance_x_mm,
                fov,
                image_width);

            n = ((projected_height-1-iy)*projected_width+ix);
            if ((x > -1) && (x < image_width)) {
                projected_lookup[n] = (y*image_width+x)*3;
                reprojected_lookup[y*image_width+x] = n*3;
            }
        }
    }


    n=0;
    for (iy=0;iy<image_height;iy++) {
        prev_n2=0;
        curr_y=0;
        for (ix=0;ix<image_width;ix++,n++,n2+=3) {
            n1 = reprojected_lookup[n];
            if (n1>0) {
                curr_y = iy;
                if ((prev_n2>0)&&(prev_y>0)) {
                    for (i=prev_n2;i<n2;i+=3) {
                        for (j=0;j<curr_y-prev_y;j++) {
                            n3 = (i-j*image_width*3)/3;
                            reprojected_lookup[n3] = n1;
                        }
                    }
                }
                prev_n2 = n2;
            }
        }
        if (curr_y>0) {
            prev_y=curr_y;
        }
    }

}

/* find pixels which probably belong to the road */
void road_segment(
    int image_width,
    int image_height,
    int sample_width,
    int threshold,
    unsigned char * image_data,
    unsigned char * image_data_buffer,
    unsigned char * image_segmented)
{
    int i,x,y,reflectance,Y,U,V,r,g,b,start_x,end_x;
    int reference, luminance;

    for (i=0;i<image_width*image_height*3;i+=3) {
        if (!((image_data[i]==0) &&
              (image_data[i+1]==0) &&
              (image_data[i+2]==0))) {
            r = image_data[i];
            g = image_data[i+1];
            b = image_data[i+2];
            Y = MIN(
                ABS(r*2104 + g*4130 + b*802 + 4096 + 131072) >> 13, 235);
            if (g*2<r+b+20) {
                U = MIN(
                    ABS(r*-1214 + g*-2384 + b*3598 + 4096 + 1048576) >> 13,
                    240);
                V = MIN(
                    ABS(r*3598 + g*-3013 + b*-585 + 4096 + 1048576) >> 13,
                    240);
                reflectance = (U*16>>3)+(V>>3);
            }
            else {
                reflectance=0;
            }

            image_data_buffer[i]=reflectance;
            image_data_buffer[i+1]=0;
            image_data_buffer[i+2]=Y;
        }
        else {
            image_data_buffer[i]=0;
            image_data_buffer[i+1]=0;
            image_data_buffer[i+2]=0;
        }
    }

    memcpy(
        (void*)image_segmented,
        (void*)image_data_buffer,
        image_width*image_height*3);

    start_x = (image_width/2) - (sample_width/2);
    end_x = start_x + sample_width;
    for (x=start_x;x<end_x;x++) {
        for (y=image_height-1;y >=image_height-10;y--) {
            i = (y*image_width+x)*3;
            reference = image_segmented[i];
            if (reference!=0) {
                luminance = image_segmented[i+2];
                for (i=0;i<image_width*image_height*3;i+=3) {
                    if (image_segmented[i]==reference) {
                        if (ABS(image_segmented[i+2]-luminance)
                            < threshold) {
                            image_segmented[i]=0;
                            image_segmented[i+1]=255;
                        }
                    }
                }
            }
        }
    }
}

/* update the ground plane image */
void road_project(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int projected_width,
    int projected_height,
    int * projected_lookup,
    unsigned char * projected)
{
    int ix,iy,n0,n1;

    for (iy=0;iy<projected_height;iy++) {
        for (ix=0;ix<projected_width;ix++) {
            n0 = (projected_height-1-iy)*projected_width+ix;
            n1 = projected_lookup[n0];
            n0*=3;
            if (n1>0) {
                projected[n0] = image_data[n1];
                projected[n0+1] = image_data[n1+1];
                projected[n0+2] = image_data[n1+2];
            }
            else {
                projected[n0] = 0;
                projected[n0+1] = 0;
                projected[n0+2] = 0;
            }
        }
    }
}

/* reproject from the ground plane image back to the original image */
void road_reproject(
    int projected_width,
    int projected_height,
    unsigned char * image_ground_plane,
    int image_width,
    int image_height,
    int * reprojected_lookup,
    unsigned char * image_data)
{
    int ix,iy,n1,n2=0,n=0,r,g,b;

    for (iy=0;iy<image_height;iy++) {
        for (ix=0;ix<image_width;ix++,n++,n2+=3) {
            n1 = reprojected_lookup[n];
            if (n1>0) {
                r = image_ground_plane[n1];
                g = image_ground_plane[n1+1];
                b = image_ground_plane[n1+2];

                image_data[n2] =
                    (r*3+image_data[n2]*5)>>3;
                image_data[n2+1] =
                    (g*3+image_data[n2+1]*5)>>3;
                image_data[n2+2] =
                    (b*3+image_data[n2+2]*5)>>3;
            }
        }
    }
}

