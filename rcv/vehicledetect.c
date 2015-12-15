/*
 detection of road vehicles
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

#include "vehicledetect.h"

/* creates an image containing possible shadows */
void vehicledetect_shadows(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int ty,
    int by,
    unsigned short * road_lines,
    int * reprojected_lookup,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    unsigned char * image_shadows,
    int layer,
    int reflectance_threshold_percent)
{
    int i,x,y,n,av_reflectance=0,hits,start_x,end_x,dx,dy,shadow_threshold;

    hits=0;
    /* for each y coordinate */
    for (y=ty;y<by;y++) {
        start_x=0;
        end_x = image_width;
        /* get the start and end x coordinates from the lane detection */
        if (road_lines[0]>0) {
            dy = road_lines[3] - road_lines[1];
            if (dy!=0) {
                dx = road_lines[2] - road_lines[0];
                start_x =
                    road_lines[0] +
                    MULDIVINT32((y-road_lines[1]),dx,dy);
            }
            dy = road_lines[7] - road_lines[5];
            if (dy!=0) {
                dx = road_lines[6] - road_lines[4];
                end_x = road_lines[4] + MULDIVINT32((y-road_lines[5]),dx,dy);
            }
        }
        n = y*image_width + start_x;
        i = n*3;
        for (x=start_x;x<end_x;x++,i+=3,n++) {
            /* does segmentation indicate that this is a road pixel? */
            if ((reprojected_lookup[n]>0) &&
                (segmented[reprojected_lookup[n]+1]==255)) {
                /* update the average road reflectance */
                av_reflectance += image_data[i+2];
                hits++;
            }
        }
    }
    if (hits>0) {
        /* calculate average road reflectance */
        av_reflectance /= hits;

        /* shadow threshold */
        shadow_threshold =
            MULDIVINT32(av_reflectance,reflectance_threshold_percent,100);

        /* for each y coordinate */
        for (y=ty;y<by;y++) {
            start_x=0;
            end_x = image_width;
            /* get the start and end x coordinates from the lane detection */
            if (road_lines[0]>0) {
                dy = road_lines[3] - road_lines[1];
                if (dy!=0) {
                    dx = road_lines[2] - road_lines[0];
                    start_x =
                        road_lines[0] + MULDIVINT32((y-road_lines[1]),dx,dy);
                }
                dy = road_lines[7] - road_lines[5];
                if (dy!=0) {
                    dx = road_lines[6] - road_lines[4];
                    end_x =
                        road_lines[4] + MULDIVINT32((y-road_lines[5]),dx,dy);
                }
            }
            i=(y*image_width+start_x)*3;
            for (x=start_x;x<end_x;x++,i+=3) {
                /* is this pixel below the shadow threshold? */
                if (image_data[i+2] < shadow_threshold) {
                    if (ABS(image_data[i+2]-image_data[i]) < 20) {
                        if (ABS(image_data[i+1]-image_data[i]) < 20) {
                            /* mark as possible shadow */
                            image_shadows[i+layer] = 255;
                        }
                    }
                }
            }
        }
    }
}

int vehicledetect_validate_region(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    int * reprojected_lookup,
    int tx,
    int ty,
    int bx,
    int by,
    int region_threshold)
{
    int validated=0;
    int segmented_tx_top,segmented_bx_top;
    int segmented_tx_bottom,segmented_bx_bottom;
    int segmented_ty,segmented_by;
    int x,y,start_x,end_x,n,width_top,width_bottom;
    int hits=0,vehicle_pixels=0;
    int cx = projected_width>>1;

    ty = by - (bx-tx);
    if (ty<0) ty=0;

    segmented_by =
        (reprojected_lookup[by*image_width+tx]/3)/projected_width;
    segmented_tx_bottom =
        (reprojected_lookup[by*image_width+tx]-
         segmented_by*projected_width*3)/3;
    segmented_bx_bottom =
        (reprojected_lookup[by*image_width+bx]-
         segmented_by*projected_width*3)/3;
    width_bottom = segmented_bx_bottom - segmented_tx_bottom;
    segmented_ty=0;
    segmented_tx_top = cx +
      (segmented_tx_bottom - cx)*projected_height/
      (projected_height-1-segmented_by);
    segmented_bx_top = cx +
      (segmented_bx_bottom - cx)*projected_height/
      (projected_height-1-segmented_by);
    width_top = segmented_bx_top - segmented_tx_top;

    for (y=segmented_ty;y<segmented_by;y++) {
        start_x = segmented_tx_top +
            (y-segmented_ty)*(segmented_tx_bottom-segmented_tx_top)/
            (segmented_by-segmented_ty);
        end_x = start_x + width_top +
            (y-segmented_ty)*(width_bottom-width_top)/
            (segmented_by-segmented_ty);
        if (start_x<0) start_x = 0;
        if (end_x > projected_width) end_x = projected_width;
        n = (y*projected_width + start_x)*3;
        for (x = start_x; x < end_x; x++, n+=3) {
            if (segmented[n+1]==0) {
                vehicle_pixels++;
            }
            hits++;
        }
    }
    if (hits>0) {
        vehicle_pixels = MULDIVINT32(vehicle_pixels,100,hits);
    }

    if (vehicle_pixels>=region_threshold) {
        validated=1;
    }
    return validated;
}

/* Is there some difference between the left and right sides
   of the vehicle and the background? (strong vertical edges) */
int vehicledetect_validate_vehicle_sides(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int tx,
    int ty,
    int bx,
    int by,
    int vehicle_sides_threshold)
{
    int y,n0,n1,edges_left=0,edges_right=0;
    int stride = image_width*3;
    int validated=0;

    if ((tx > 5) && (bx < image_width-6)) {
        vehicle_sides_threshold *= (by-ty)*2;
        n0 = (ty*image_width+tx)*3;
        n1 = (ty*image_width+bx)*3;
        for (y=ty;y<by;y++,n0+=stride,n1+=stride) {
            edges_left +=
                ABS(image_data[n0-9]+image_data[n0-15] -
                    image_data[n0+9] - image_data[n0+15]);
            edges_right +=
                ABS(image_data[n1-9]+image_data[n1-15] -
                    image_data[n1+9]-image_data[n1+15]);
        }
        if ((edges_left >= vehicle_sides_threshold) &&
            (edges_right >= vehicle_sides_threshold)) {
            validated = 1;
        }
    }

    return validated;
}

/* does the region specified look like a vehicle? */
int vehicledetect_validate(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * segmented,
    int * reprojected_lookup,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int tx,
    int ty,
    int bx,
    int by,
    int slope_tollerance,
    int vehicle_sides_threshold,
    int region_threshold,
    int layer,
    int range_mm,
    int max_range_mm)
{
    int x,y,n,av_left=0,av_right=0,hits_left=0,hits_right=0;
    int av_top=0, av_bottom=0, hits_top, hits_bottom;
    int cx = tx + ((bx-tx)/2);
    int cy = ty + ((by-ty)/2);
    int stride = image_width*3;
    int validated=0;
    int width_samples = (bx-tx)>>2;

    if (width_samples > 2) {

        /* test the average height to ensure this isn't just a thin line*/
        hits_left=0;
        for (x=tx;x<bx;x+=4) {
            n = (by*image_width+x)*3;
            for (y=by;y>ty;y--,n-=stride) {
                if (image_shadows[n+layer]>0) {
                    hits_left++;
                }
            }
        }

        if (hits_left/width_samples > MIN_AVERAGE_HEIGHT) {

            /* test whether the bottom of the region looks horizontal */
            av_left=av_right=hits_left=hits_right=0;
            for (x=tx;x<bx;x+=4) {
                n = (by*image_width+x)*3;
                for (y=by;y>ty;y--,n-=stride) {
                    if (image_shadows[n+layer]>0) {
                        break;
                    }
                }
                if (y!=ty) {
                    if (x<cx) {
                        av_left+=y;
                        hits_left++;
                    }
                    else {
                        av_right+=y;
                        hits_right++;
                    }
                }
            }
            if (hits_left>0) {
                av_left/=hits_left;
            }
            if (hits_right>0) {
                av_right/=hits_right;
            }
            if (ABS(av_left - av_right) < slope_tollerance) {

                /* test whether the top of the region looks horizontal */
                av_left=av_right=hits_left=hits_right=0;
                for (x=tx;x<bx;x+=4) {
                    n = (ty*image_width+x)*3;
                    for (y=ty;y<by;y++,n+=stride) {
                        if (image_shadows[n+layer]>0) {
                            break;
                        }
                    }
                    if (y!=by) {
                        if (x<cx) {
                            av_left+=y;
                            hits_left++;
                        }
                        else {
                            av_right+=y;
                            hits_right++;
                        }
                    }
                }
                if (hits_left>0) {
                    av_left/=hits_left;
                }
                if (hits_right>0) {
                    av_right/=hits_right;
                }
                if (ABS(av_left - av_right) < slope_tollerance) {

                    /* test whether the left side of the
                       region looks vertical */
                    av_top=av_bottom=hits_top=hits_bottom=0;
                    for (y=ty; y<by; y++) {
                        n = (y*image_width+tx)*3;
                        for (x=tx;x<cx;x++,n+=3) {
                            if (image_shadows[n+layer]>0) {
                                break;
                            }
                        }
                        if (x!=cx) {
                            if (y < cy) {
                                av_top+=x;
                                hits_top++;
                            }
                            else {
                                av_bottom+=x;
                                hits_bottom++;
                            }
                        }
                    }
                    if (hits_top) {
                        av_top /= hits_top;
                    }
                    if (hits_bottom) {
                        av_bottom /= hits_bottom;
                    }
                    if ((hits_top==0) ||
                        (hits_bottom==0) ||
                        (ABS(av_top - av_bottom) < slope_tollerance)) {

                        /* test whether the right side of the
                           region looks vertical */
                        av_top=av_bottom=hits_top=hits_bottom=0;
                        for (y=ty; y<by; y++) {
                            n = (y*image_width+bx)*3;
                            for (x=bx;x>cx;x--,n-=3) {
                                if (image_shadows[n+layer]>0) {
                                    break;
                                }
                            }
                            if (x!=cx) {
                                if (y < cy) {
                                    av_top+=x;
                                    hits_top++;
                                }
                                else {
                                    av_bottom+=x;
                                    hits_bottom++;
                                }
                            }
                        }
                        if (hits_top) {
                            av_top /= hits_top;
                        }
                        if (hits_bottom) {
                            av_bottom /= hits_bottom;
                        }
                        if ((hits_top==0) ||
                            (hits_bottom==0) ||
                            (ABS(av_top - av_bottom) < slope_tollerance)) {

                            validated =
                                vehicledetect_validate_vehicle_sides(
                                    image_width, image_height,
                                    image_data, image_shadows,
                                    tx, ty, bx, by,
                                    vehicle_sides_threshold);
                            if (validated!=0) {
                                if (range_mm < max_range_mm*40/100) {
                                    validated =
                                        vehicledetect_validate_region(
                                            image_width, image_height,
                                            projected_width,
                                            projected_height,
                                            segmented, reprojected_lookup,
                                            tx, ty,bx, by,
                                            region_threshold);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return validated;
}

/* returns the stopping distance in millimetres for the given speed */
int vehicledetect_stopping_distance(
    int speed_mph)
{
    return ((speed_mph*speed_mph/20)+speed_mph)*100000/328;
}

void vehicledetect_show_vehicles(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int r, int g, int b)
{
    int i,tx,ty,bx,by,x,y,n,range_mm,xx;
    int stride = image_width*3;

    for (i=0;i<no_of_possible_vehicles;i++) {
        /* is this a valid observation, consistent
           with recent observations? */
        if (possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_VALID]!=0) {
            tx = possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_TX];
            /* has not been set to zero */
            if (tx>0) {
                bx = possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_BX];
                by = possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_BY];
                ty = by - ((bx-tx)>>1);
                if (ty<0) ty=0;
                range_mm = possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_RANGE];

                xx = tx + ((bx-tx)>>1)-12;
                if (range_mm >= 10000) xx-=8;
                if (range_mm >= 100000) xx-=8;
                numberimages_draw_decimal(
                    image_width,image_height, image_data,
                    range_mm, 1000, 1, 1,
                    xx, by+2, 255,255,255);

                /* draw verticals */
                for (y=ty;y<=by;y++) {
                    n = (y*image_width+tx)*3;
                    image_data[n]=r;
                    image_data[n+1]=g;
                    image_data[n+2]=b;
                    image_data[n+3]=r;
                    image_data[n+4]=g;
                    image_data[n+5]=b;
                    n = (y*image_width+bx)*3;
                    image_data[n]=r;
                    image_data[n+1]=g;
                    image_data[n+2]=b;
                    image_data[n-3]=r;
                    image_data[n-2]=g;
                    image_data[n-1]=b;
                }
                /* draw horizontals */
                for (x=tx;x<=bx;x++) {
                    n = (by*image_width+x)*3;
                    image_data[n]=r;
                    image_data[n+1]=g;
                    image_data[n+2]=b;
                    image_data[n-stride]=r;
                    image_data[n+1-stride]=g;
                    image_data[n+2-stride]=b;
                }
            }
        }
    }
}

/* adds an observation of a possible vehicle */
void vehicledetect_add_observation(
    int * possible_vehicles,
    int index,
    int tx,
    int ty,
    int bx,
    int by,
    int range_mm)
{
    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_TX] = tx;

    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_TY] = ty;

    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_BX] = bx;

    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_BY] = by;

    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS+VEHICLES_FIELD_RANGE] = range_mm;

    possible_vehicles[
        index*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_VALID] = 1;
}

/* From an image containing possible shadows detect regions and test
   them for similarity to a possible vehicle */
int vehicledetect_shadow_regions(
    int image_width,
    int image_height,
    unsigned char * segmented,
    unsigned char * image_data,
    unsigned char * image_shadows,
    int ground_plane_width_mm,
    int vehicle_width_mm,
    int min_range_mm,
    int max_range_mm,
    int vehicle_max_range_ahead_mm,
    int * reprojected_lookup,
    int projected_width,
    int projected_height,
    int * possible_vehicles,
    unsigned short * stack,
    int layer,
    int horizon,
    int bonnet,
    int * range_lookup)
{
    int x,y,i,tx=0,ty=0,bx=0,by=0,range_mm,n;
    int no_of_possible_vehicles = 0;
    int stack_size = (image_width>>2)*(image_height>>2)*2;

    i=horizon*image_width*3;
    for (y=horizon;y<bonnet-2;y++) {
        for (x=0;x<image_width;x++,i+=3) {

            /* does a shadow pixel exist here? */
            if (image_shadows[i+layer]==255) {

                /* fill the shadow area */
                floodfill_fill_layer(
                    x, y, image_width, image_height,
                    stack, stack_size, image_shadows,
                    layer, 255, 254,
                    &tx, &ty, &bx, &by);

                /* we're only interested in vehicles
                   which are fairly close */
                n = (image_height-by)*3;
                range_mm = range_lookup[n];
                if (range_mm < vehicle_max_range_ahead_mm) {

                    /* what is the expected width in pixels for a
                       vehicle at this range? */
                    if ((bx-tx > range_lookup[n+1]) &&
                        (bx-tx < range_lookup[n+2])) {

                        /* do the pixels inside the bounding box look
                           like a rectangle? */
                        if (vehicledetect_validate(
                            image_width, image_height,
                            projected_width,projected_height,
                            segmented, reprojected_lookup,
                            image_data,
                            image_shadows,
                            tx, ty, bx, by,
                            MAX_SLOPE,
                            VEHICLEDETECT_SIDE_EDGES,
                            VEHICLEDETECT_MIN_REGION_VEHICLE_PERCENT,
                            layer,range_mm,max_range_mm)!=0) {

                            /* store this region
                               as a possible vehicle */
                            vehicledetect_add_observation(
                                possible_vehicles,
                                no_of_possible_vehicles,
                                tx,ty,bx,by,range_mm);

                            no_of_possible_vehicles++;

                            /* abort if the maximum number of
                               vehicles is reached */
                            if (no_of_possible_vehicles==
                                MAX_POSSIBLE_VEHICLES) {
                                x=image_width;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return no_of_possible_vehicles;
}

/* Checks to see if a similar vehicle observation has been made recently */
int vehicledetect_online_validation(
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int * possible_vehicles_history,
    int range_tollerance_mm)
{
    int i,j,range1_mm,range2_mm,x1,x2;
    int observation_consistency,max_consistency,winner=-1;
    int validated=0;

    max_consistency = POSSIBLE_VEHICLES_MIN_OBSERVATIONS;

    /* for each possible vehicle observed */
    for (i = no_of_possible_vehicles-1; i >= 0; i--) {
        /* range to the observed vehicle */
        range1_mm =
            possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_RANGE];
        x1 = possible_vehicles[
                 i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_TX] +
            ((possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_BX]-
              possible_vehicles[
                 i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_TX])>>1);
        observation_consistency=0;
        /* for each previous observation */
        for (j = POSSIBLE_VEHICLES_HISTORY-1; j >= 0 ; j--) {
            /* range to a previously observed vehicle */
            range2_mm =
                possible_vehicles_history[
                    j*POSSIBLE_VEHICLES_HISTORY_FIELDS +
                    POSSIBLE_VEHICLES_HISTORY_FIELD_RANGE];
            /* are these range values similar? */
            if ((range2_mm > 0) &&
                (ABS(range2_mm - range1_mm) < range_tollerance_mm)) {
                /* vehicle in a similar horizontal position */
                x2 = possible_vehicles_history[
                    j*POSSIBLE_VEHICLES_HISTORY_FIELDS +
                    POSSIBLE_VEHICLES_HISTORY_FIELD_CX];
                if (ABS(x2-x1) < VEHICLES_HORIZONTAL_TOLLERANCE) {
                    /* increment the consistency count */
                    observation_consistency++;
                    if (observation_consistency >= max_consistency) {
                        /* best result so far */
                        max_consistency = observation_consistency;
                        winner=i;
                        validated = 1+i;
                    }
                }
            }
        }
    }

    /* clear valid flag for inconsistent observations */
    for (i = no_of_possible_vehicles-1; i >= 0; i--) {
        if (i != winner) {
            possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_VALID]=0;
        }
    }
    return validated;
}

/* shuffle the historical observations buffer by one frame */
void vehicledetect_shuffle_history(
    int * possible_vehicles_history)
{
    int i;

    for (i = POSSIBLE_VEHICLES_HISTORY-1; i > 0; i--) {
        memcpy(
            (void*)(&possible_vehicles_history[
                i*POSSIBLE_VEHICLES_HISTORY_FIELDS]),
            (void*)(&possible_vehicles_history[
                (i-1)*POSSIBLE_VEHICLES_HISTORY_FIELDS]),
            POSSIBLE_VEHICLES_HISTORY_FIELDS*sizeof(int));
    }
}

/* maintains a buffer of recent vehicle observations
   over the previous few frames */
void vehicledetect_online_update(
    int no_of_possible_vehicles,
    int * possible_vehicles,
    int * possible_vehicles_history,
    int time_interval_mS)
{
    int i,n;

    /* if no vehicles have been observed clear values to zero */
    n = no_of_possible_vehicles;
    if (no_of_possible_vehicles==0) {
        no_of_possible_vehicles = 1;
        memset(
            (void*)possible_vehicles,
            '\0', POSSIBLE_VEHICLES_FIELDS*sizeof(int));
    }

    /* update the relative times for each vehicle observation */
    for (i = POSSIBLE_VEHICLES_HISTORY_FIELD_TIME;
         i < POSSIBLE_VEHICLES_HISTORY*POSSIBLE_VEHICLES_HISTORY_FIELDS;
         i += POSSIBLE_VEHICLES_HISTORY_FIELDS) {
        possible_vehicles_history[i] += time_interval_mS;
    }

    for (i = no_of_possible_vehicles-1; i >= 0; i--) {

        vehicledetect_shuffle_history(possible_vehicles_history);

        /* insert the new observation at the start of the history */
        possible_vehicles_history[
            POSSIBLE_VEHICLES_HISTORY_FIELD_TIME] =
            time_interval_mS;

        possible_vehicles_history[
            POSSIBLE_VEHICLES_HISTORY_FIELD_RANGE] =
            possible_vehicles[
                i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_RANGE];

        possible_vehicles_history[
            POSSIBLE_VEHICLES_HISTORY_FIELD_CX] =
                possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_TX]+
                ((possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_BX]-
                possible_vehicles[
                    i*POSSIBLE_VEHICLES_FIELDS + VEHICLES_FIELD_TX])>>1);
    }

}

