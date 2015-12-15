/*
 roadcv: computer vision for road vehicles
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

#include "roadcv.h"

int roadcv_time_elapsed_mS(
    struct rcv_data * r)
{
    int elapsed_mS;
    struct timeval t;

    gettimeofday(&t, NULL);
    elapsed_mS = (int)(((t.tv_usec + 1000000 * t.tv_sec) - r->time)/1000);
    r->time = (long int)(t.tv_usec + 1000000 * t.tv_sec);
    return elapsed_mS;
}

void roadcv_open(
    int image_width,
    int image_height,
    int projected_width,
    int projected_height,
    unsigned char * image_data,
    int BGR,
    int fov,
    int tilt,
    int camera_height_mm,
    int detect_vehicles,
    int detect_signs,
    int detect_traffic_lights,
    int enable_audio,
    char * sounds_path,
    char * signs_database,
    struct rcv_data * r,
    int online,
    int headless,
    int show_lanes,
    int minimum_lane_crossing_warning_speed_mph,
    int lane_crossing_tollerance_percent)
{
    int i,shape_index,max=1;

    r->detect_traffic_lights = detect_traffic_lights;
    r->lane_crossing_tollerance_percent =
       lane_crossing_tollerance_percent;
    r->minimum_lane_crossing_warning_speed_mph =
        minimum_lane_crossing_warning_speed_mph;
    r->sounds_path = sounds_path;
    r->enable_audio = enable_audio;
    r->headless = headless;
    r->BGR = BGR;
    r->detect_vehicles = detect_vehicles;
    r->detect_signs = detect_signs;
    r->show_lanes = show_lanes;
    r->online = online;
    r->image_width = image_width;
    r->image_height = image_height;
    r->image_data = image_data;
    r->projected_width = projected_width;
    r->projected_height = projected_height;
    r->fov = fov;
    r->tilt = tilt;
    r->camera_height_mm = camera_height_mm;

    /* some default values */

    r->min_road_sign_height_mm = 1200;
    r->max_road_sign_height_mm = 3000;
    r->traffic_light_width_mm = 500;
    r->time_interval_mS=100;
    r->range_tollerance_mm=5000;
    r->minimum_symmetry = 80;
    r->minimum_sign_width=image_width*4/100;
    r->maximum_sign_width=image_width*15/100;
    r->vehicle_width_mm=2000;
    r->min_distance_mm=3000;
    r->max_distance_mm=60000;
    r->vehicle_max_range_ahead_mm=35000;
    r->segmentation_threshold=50;
    r->road_markings_edge_threshold=10;
    r->road_markings_line_threshold=20;
    r->ground_plane_width_mm = r->vehicle_width_mm*6;
    r->lane_displacement_threshold = image_width*10/100;
    r->lanes_valid = 0;
    r->horizon =
        geometry_distance_forward_pixel(
            900000,camera_height_mm,0,fov,
            image_width,image_height,tilt);
    r->bonnet =
        geometry_distance_forward_pixel(
            r->min_distance_mm,camera_height_mm,0,
            fov,image_width,image_height,tilt);
    if (r->horizon<0) r->horizon=0;
    if (r->bonnet>=image_height) r->bonnet = image_height;

    r->no_of_possible_vehicles[0] = 0;
    r->time=0;
    r->vehicle_current_range_ahead_mm = 0;
    r->stopping_distance_mm=0;
    r->warnings=0;
    r->speed_mph=0;
    r->no_of_traffic_lights[0]=0;
    r->traffic_light_state=TRAFFICLIGHT_STATE_UNKNOWN;
    r->last_warning_time=0;
    r->last_warning_icon_time=0;
    r->current_warning_icon=0;

    /* allocate memory */

    r->projected_lookup =
        (int*)malloc(r->projected_width*r->projected_height*sizeof(int));
    assert(r->projected_lookup!=0);

    r->reprojected_lookup =
        (int*)malloc(r->image_width*r->image_height*sizeof(int));
    assert(r->reprojected_lookup!=0);

    if (detect_vehicles!=0) {
        r->image_vehicles =
            (unsigned char*)malloc(
                r->image_width*r->image_height*3*sizeof(unsigned char));
        assert(r->image_vehicles!=0);
    }

    r->image_ground_plane =
        (unsigned char*)malloc(r->projected_width*r->projected_height*3);
    assert(r->image_ground_plane!=0);

    r->image_ground_plane_buffer =
        (unsigned char*)malloc(r->projected_width*r->projected_height*3);
    assert(r->image_ground_plane_buffer!=0);

    r->image_segmented =
        (unsigned char*)malloc(r->projected_width*r->projected_height*3);
    assert(r->image_segmented!=0);

    r->image_road_markings =
        (unsigned char*)malloc(r->projected_width*r->projected_height*3);
    assert(r->image_road_markings!=0);

    if (r->detect_signs!=0) {
        r->image_road_signs =
            (unsigned char*)malloc(r->image_width*r->image_height*3);
        assert(r->image_road_signs!=0);

        r->sign_shapes =
            (unsigned char*)malloc(
                (ROADSIGNS_SHAPE_TEMPLATE_SIZE*
                 ROADSIGNS_SHAPE_TEMPLATE_SIZE*ROADSIGNS_SHAPES/8)+1);
        assert(r->sign_shapes!=0);
        for (shape_index=0; shape_index<ROADSIGNS_SHAPES; shape_index++) {
            r->no_of_signs[shape_index]=0;
            switch (shape_index) {
            case ROADSIGNS_SHAPE_TRIANGLE: {
                max = ROADSIGNS_MAX_TRIANGLE;
                break;
            }
            case ROADSIGNS_SHAPE_CIRCLE: {
                max = ROADSIGNS_MAX_CIRCLE;
                break;
            }
            case ROADSIGNS_SHAPE_TRIANGLE_INVERTED: {
                max = ROADSIGNS_MAX_TRIANGLE_INVERTED;
                break;
            }
            case ROADSIGNS_SHAPE_SQUARE: {
                max = ROADSIGNS_MAX_SQUARE;
                break;
            }
            }
            r->sign_eigen_descriptor[shape_index] =
                (unsigned char*)malloc(
                    ROADSIGNS_DESCRIPTOR_SIZE*
                    ROADSIGNS_DESCRIPTOR_ENTRIES);
            assert(r->sign_eigen_descriptor[shape_index]!=0);
            r->sign_descriptor[shape_index] =
                (unsigned char*)malloc(
                    ROADSIGNS_DESCRIPTOR_SIZE*
                    ROADSIGNS_DESCRIPTOR_ENTRIES*max);
            assert(r->sign_descriptor[shape_index]!=0);
            r->sign_text[shape_index] = (char**)malloc(max*sizeof(char*));
            assert(r->sign_text[shape_index]!=0);
            r->sign_matches[shape_index] = (int*)malloc(max*sizeof(int));
            assert(r->sign_matches[shape_index]!=0);

            for (i=0; i<max; i++) {
                r->sign_text[shape_index][i]=0;
                r->sign_matches[shape_index][i]=0;
            }

            roadsigns_create_shapes(r->sign_shapes);
        }

        if (signs_database!=0) {
            roadsigns_load_database(
                signs_database,
                r->no_of_signs,
                r->sign_text,
                r->sign_shapes,
                r->sign_descriptor,
                r->sign_eigen_descriptor);
        }
    }
    if ((r->detect_signs!=0) ||
        (r->detect_vehicles!=0)) {
        for (i=0; i<SHADOW_THRESHOLDS+1; i++) {
            r->stack[i] =
                (unsigned short*)malloc(
                    (image_width>>2)*
                    (image_height>>2)*2*
                    sizeof(unsigned short));
            assert(r->stack[i]!=0);
        }
        r->regions =
            (unsigned short*)malloc(
                ROADSIGNS_MAX_REGIONS*sizeof(unsigned short));
        assert(r->regions!=0);
    }
    if (r->detect_vehicles!=0) {
        r->range_lookup =
            (int*)malloc(r->image_height*3*sizeof(int));
        assert(r->range_lookup!=0);
        geometry_create_range_lookup(
            r->image_width, r->image_height,
            r->range_lookup,
            r->horizon,
            r->camera_height_mm,
            r->fov, r->tilt,
            MULDIVINT32(r->vehicle_width_mm,60,100),
            MULDIVINT32(r->vehicle_width_mm,180,100));

        for (i=0; i<SHADOW_THRESHOLDS; i++) {
            r->possible_vehicles[i] =
                (int*)malloc(
                    MAX_POSSIBLE_VEHICLES*
                    POSSIBLE_VEHICLES_FIELDS*
                    sizeof(int));
            assert(r->possible_vehicles[i]!=0);
        }
        r->possible_vehicles_history =
            (int*)malloc(
                POSSIBLE_VEHICLES_HISTORY_FIELDS*
                POSSIBLE_VEHICLES_HISTORY*sizeof(int));
        assert(r->possible_vehicles_history!=0);
        for (i=0;
             i < POSSIBLE_VEHICLES_HISTORY_FIELDS*POSSIBLE_VEHICLES_HISTORY;
             i++) {
            r->possible_vehicles_history[i]=0;
        }
    }

    /* create lookup tables for ground plane projection */
    road_create_lookups(
        r->image_width, r->image_height,
        r->min_distance_mm, r->max_distance_mm,
        r->camera_height_mm, r->fov, r->tilt,
        r->vehicle_width_mm,
        r->projected_width, r->projected_height,
        r->projected_lookup, r->reprojected_lookup);

    if (r->detect_traffic_lights!=0) {
        r->traffic_light_history =
            (unsigned short*)malloc(
                TRAFFICLIGHTS_HISTORY*TRAFFICLIGHTS_HISTORY_FIELDS*
                sizeof(unsigned short));
        assert(r->traffic_light_history!=0);
        memset((void*)(r->traffic_light_history),'\0',
            TRAFFICLIGHTS_HISTORY*TRAFFICLIGHTS_HISTORY_FIELDS*
            sizeof(unsigned short));
        r->image_traffic_lights =
            (unsigned char*)malloc(r->image_width*r->image_height*3);
        assert(r->image_traffic_lights!=0);
        for (i=0;i<TRAFFICLIGHTS_THRESHOLDS;i++) {
            r->stack_traffic_lights[i] =
                (unsigned short*)malloc(
                    (image_width>>2)*
                    (image_height>>2)*2*
                    sizeof(unsigned short));
            assert(r->stack_traffic_lights[i]!=0);
            r->traffic_lights[i]=
                (unsigned short*)malloc(
                    TRAFFICLIGHTS_MAX*TRAFFICLIGHTS_FIELDS*
                    sizeof(unsigned short));
            assert(r->traffic_lights[i]!=0);
        }
    }
    if ((r->detect_traffic_lights!=0) ||
        (r->detect_signs!=0)) {
        assert(r->image_height - r->horizon>0);
        r->height_lookup =
            (int*)malloc((r->image_height-r->horizon)*3*sizeof(int));
        assert(r->height_lookup!=0);

        geometry_create_height_lookup(
            r->image_width, r->image_height,
            r->height_lookup,
            r->min_road_sign_height_mm,
            r->max_road_sign_height_mm,
            r->traffic_light_width_mm,
            r->horizon,
            r->camera_height_mm,
            r->fov, r->tilt);
    }
}

void roadcv_close(struct rcv_data * r) {

    int i,j;

    /* free allocated memory */

    if (r->detect_vehicles!=0) free (r->image_vehicles);
    free (r->projected_lookup);
    free (r->reprojected_lookup);
    free (r->image_ground_plane);
    free (r->image_ground_plane_buffer);
    free (r->image_segmented);
    free (r->image_road_markings);
    if (r->detect_signs!=0) {
        free (r->image_road_signs);
        free (r->sign_shapes);
        for (i=0; i<ROADSIGNS_SHAPES; i++) {
            for (j=0; j<r->no_of_signs[i]; j++) {
                free (r->sign_text[i][j]);
            }
            free (r->sign_text[i]);
            free (r->sign_eigen_descriptor[i]);
            free (r->sign_descriptor[i]);
        }
    }
    if (r->detect_traffic_lights!=0) {
        free(r->traffic_light_history);
        free(r->image_traffic_lights);
        for (i=0;i<TRAFFICLIGHTS_THRESHOLDS;i++) {
            free(r->stack_traffic_lights[i]);
            free(r->traffic_lights[i]);
        }
    }
    if ((r->detect_traffic_lights!=0) ||
        (r->detect_signs!=0)) {
        free(r->height_lookup);
    }
    if ((r->detect_signs!=0) || (r->detect_vehicles!=0)) {
        for (i=0; i<SHADOW_THRESHOLDS+1; i++) {
            free (r->stack[i]);
        }
        free (r->regions);
    }
    if (r->detect_vehicles!=0) {
        for (i=0; i<SHADOW_THRESHOLDS; i++) {
            free (r->possible_vehicles[i]);
        }
        free (r->possible_vehicles_history);
        free (r->range_lookup);
    }
}

/* detect traffic lights */
void roadcv_lights(struct rcv_data * r)
{
    int i,j,thresh,layer;
    int reflectance_threshold, variance_threshold;
    int state_estimate,red_votes,amber_votes,green_votes;
    int previous_state;
    unsigned short * lanes = &(r->road_lines[8]);

    if (r->online!=0) {
        lanes = r->lanes_online;
    }

    if (r->detect_traffic_lights!=0) {
        /* clear the image used for filtering */
        memset((void*)r->image_traffic_lights,'\0',
            r->image_width*r->image_height*3);

        red_votes=amber_votes=green_votes=0;

        /* search for traffic lights using a couple of thresholds */
#pragma omp parallel for
        for (layer = 0; layer < TRAFFICLIGHTS_THRESHOLDS; layer++) {

            /* choose the appropriate threshold */
            if (layer==0) {
                reflectance_threshold=TRAFFICLIGHTS_REFLECTANCE_THRESHOLD1;
                variance_threshold=TRAFFICLIGHTS_VARIANCE_THRESHOLD1;
            }
            else {
                reflectance_threshold=TRAFFICLIGHTS_REFLECTANCE_THRESHOLD2;
                variance_threshold=TRAFFICLIGHTS_VARIANCE_THRESHOLD2;
            }

            trafficlights_filter(
                r->image_width, r->image_height,
                r->image_data, r->image_traffic_lights,
                r->height_lookup,
                r->horizon,
                layer,
                lanes,
                reflectance_threshold,
                variance_threshold);

            r->no_of_traffic_lights[layer] =
                trafficlights_detect(
                    r->image_width, r->image_height,
                    r->image_data,
                    r->image_traffic_lights,
                    r->horizon,
                    layer,
                    r->stack_traffic_lights[layer],
                    lanes,
                    r->traffic_lights[layer]);
        }

        /* collate the results together */
        for (thresh=1;thresh<TRAFFICLIGHTS_THRESHOLDS;thresh++) {
            for (i = 0; i < r->no_of_traffic_lights[thresh]; i++) {
                if (r->no_of_traffic_lights[0]==TRAFFICLIGHTS_MAX) break;
                for (j = 0; j < TRAFFICLIGHTS_FIELDS; j++) {
                    r->traffic_lights[0][
                        r->no_of_traffic_lights[0]*TRAFFICLIGHTS_FIELDS+j]=
                        r->traffic_lights[thresh][i*TRAFFICLIGHTS_FIELDS+j];
                }
                r->no_of_traffic_lights[0]++;
            }
        }

        /* do the lights indicate we should stop? */
        for (i=0; i < r->no_of_traffic_lights[0]; i++) {
            switch (r->traffic_lights[0][i*TRAFFICLIGHTS_FIELDS+
                TRAFFICLIGHTS_FIELD_COLOUR]) {
                case TRAFFICLIGHT_STATE_GREEN: {
                    green_votes++;
                    break;
                }
                case TRAFFICLIGHT_STATE_RED: {
                    red_votes++;
                    break;
                }
                case TRAFFICLIGHT_STATE_AMBER: {
                    amber_votes++;
                    break;
                }
            }
        }

        /* what is the current traffic light state? */
        state_estimate = TRAFFICLIGHT_STATE_UNKNOWN;
        if ((green_votes > amber_votes) &&
            (green_votes > red_votes)) {
            state_estimate = TRAFFICLIGHT_STATE_GREEN;
        }
        else {
            if ((red_votes > amber_votes) &&
                (red_votes > green_votes)) {
                state_estimate = TRAFFICLIGHT_STATE_RED;
            }
            else {
                if ((amber_votes > red_votes) &&
                    (amber_votes > green_votes)) {
                    state_estimate = TRAFFICLIGHT_STATE_AMBER;
                }
            }
        }

        /* record the previous state */
        previous_state = r->traffic_light_state;

        /* update the current state */
        if (r->online==0) {
            r->traffic_light_state = state_estimate;
        }
        else {
            r->traffic_light_state =
                trafficlights_online_update(
                    state_estimate, r->traffic_light_history, 1);
        }

        if (state_estimate != TRAFFICLIGHT_STATE_UNKNOWN) {
            /* stop warning */
            if (((r->traffic_light_state==TRAFFICLIGHT_STATE_RED) ||
                 (r->traffic_light_state==TRAFFICLIGHT_STATE_AMBER)) &&
                ((previous_state==TRAFFICLIGHT_STATE_UNKNOWN) ||
                 (previous_state==TRAFFICLIGHT_STATE_GREEN))) {
                r->warnings |= WARNING_STOP_LIGHT;
            }
            else {
                /* go warning */
                if ((r->traffic_light_state==TRAFFICLIGHT_STATE_GREEN) &&
                    (previous_state!=TRAFFICLIGHT_STATE_GREEN)) {
                    r->warnings |= WARNING_GO;
                }
            }
        }
    }
}

/* detect road signs */
void roadcv_signs(struct rcv_data * r)
{

    static unsigned char road_sign_colour_model[] = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,24,0,0,0,252,1,
        0,0,252,3,0,0,254,3,0,0,255,1,0,0,63,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
    };

    r->sign_type=-1;
    if (r->detect_signs!=0) {

        r->sign_region[0]=0;

        roadsigns_detect(
            r->image_width, r->image_height,
            r->image_data,
            r->horizon,
            r->minimum_sign_width,
            r->maximum_sign_width,
            r->minimum_symmetry,
            r->no_of_signs,
            r->sign_shapes,
            r->sign_descriptor,
            r->sign_eigen_descriptor,
            road_sign_colour_model,
            r->image_road_signs,
            &(r->shape_index),
            &(r->sign_index),
            r->sign_matches,
            &(r->sign_region[0]),
            &(r->sign_region[1]),
            &(r->sign_region[2]),
            &(r->sign_region[3]),
            r->stack[SHADOW_THRESHOLDS],
            r->regions);
    }
}

/* updates the lane crossing warning */
void roadcv_lane_crossing_warning(struct rcv_data * r) {
    int cx, variance;

    if ((r->online!=0) &&
        (r->speed_mph >= r->minimum_lane_crossing_warning_speed_mph)) {

        /* Calculate the maximum variance from the centre of the image.
           If a lane marking is inside this region then lane crossing
           is assumed */
        cx = r->image_width>>1;
        variance =
            MULDIVINT32(
                r->lane_crossing_tollerance_percent,r->image_width,200);

        /* check the coordinates of the lane markings */
        if ((r->lanes_online[2] > cx - variance) ||
            (r->lanes_online[6] < cx + variance)) {
            r->warnings |= WARNING_LANE_CROSSING;
        }
    }
}

/* detect road markings */
void roadcv_markings(struct rcv_data * r) {

    int i,lane_displacement,suspend_update=0;
    int av_lane_width, min_slope;

    if (r->online==0) {
        r->lanes_valid=LANES_VALID_FRAMES;
    }

    av_lane_width =
        roadmarkings_detect(
            r->projected_width, r->projected_height,
            r->vehicle_width_mm, r->ground_plane_width_mm,
            r->road_markings_edge_threshold,
            r->image_ground_plane,
            r->image_segmented,
            r->image_road_markings,
            r->road_lines,
            r->road_markings_line_threshold,
            r->image_width,
            r->projected_lookup);

    if ((av_lane_width>10) &&
        (r->road_lines[0]!=0) &&
        (r->road_lines[4]!=0)) {

        /* reproject the lines from the ground plane
           back into the original image coordinates */
        roadmarkings_reproject_lanes(
            r->image_width,r->image_height,
            r->image_data,
            r->projected_width,
            r->projected_height,
            r->min_distance_mm,
            r->max_distance_mm,
            r->road_lines,
            &(r->road_lines[8]),
            r->projected_lookup,
            r->min_distance_mm + 4000);

        /* Calculate minimum slope for the reprojected lane markings.
           For example, we don't expect the lanes to look parallel,
           or diverging towards the horizon */
        min_slope =
            MULDIVINT32(r->image_width,ROADMARKINGS_MINIMUM_SLOPE,100);

        if ((r->road_lines[8]>0) &&
            (r->road_lines[12]>0) &&
            (r->road_lines[12]<r->image_width) &&
            (r->road_lines[8]-r->road_lines[10] > min_slope) &&
            (r->road_lines[14]-r->road_lines[12] > min_slope)) {

            if (r->online != 0) {
                lane_displacement = 0;
                for (i=0; i<8; i+=2) {
                    lane_displacement +=
                        ABS(r->lanes_online[i] - r->road_lines[8+i]);
                }
                lane_displacement >>= 2;
                if (lane_displacement < r->lane_displacement_threshold) {
                    if (r->lanes_valid < LANES_VALID_FRAMES*2) {
                        r->lanes_valid++;
                    }
                }
                else {
                    if (r->lanes_valid>0) {
                        r->lanes_valid--;
                        suspend_update=1;
                    }
                }
            }
        }
        else {
            for (i=0; i<16; i++) r->road_lines[i]=0;
            r->lanes_valid=0;
        }
    }
    else {
        r->lanes_valid=0;
    }

    if ((r->online==0) ||
        ((r->online!=0) && (suspend_update==0))) {
        /* for each lane marking point */
        for (i=0; i < 8; i++) {
            if (r->lanes_valid > 0) {
                /* incrementally update the lane position */
                r->lanes_online[i] +=
                    MULDIVINT32(
                        (r->road_lines[8+i] - r->lanes_online[i]),2,10);
            }
            else {
                /* online version is the same */
                r->lanes_online[i] = r->road_lines[8+i];
            }
        }
    }

    /* lane crossing warning */
    roadcv_lane_crossing_warning(r);
}

void roadcv_BGRtoRGB(

    int image_width,
    int image_height,
    unsigned char * image_data)
{
    int i;
    unsigned char r,b;

    for (i=0; i<image_width*image_height*3; i+=3) {
        b = image_data[i];
        r = image_data[i+2];
        image_data[i] = r;
        image_data[i+2] = b;
    }
}

void roadcv_draw_lanes(struct rcv_data * r) {

    unsigned short * lines;
    int rr,gg,bb;

    if (r->show_lanes!=0) {
        if ((r->online==0) || ((r->online!=0) &&
            (r->lanes_valid>=LANES_VALID_FRAMES))) {
            if (r->online==0) {
                lines = &(r->road_lines[8]);
            }
            else {
                lines = r->lanes_online;
            }
            if (r->warnings & WARNING_LANE_CROSSING) {
                rr=255;
                gg=bb=0;
            }
            else {
                rr=bb=0;
                gg=255;
            }

            roadmarkings_draw_lanes(
                r->image_width, r->image_height,
                r->image_data,
                lines,
                rr, gg, bb, 8);
        }
    }
}

void roadcv_vehicles(struct rcv_data * r) {

    int shadow_threshold_percent, online_validated, layer,i,j;
    unsigned short * lanes = &(r->road_lines[8]);

    if ((r->detect_vehicles!=0) &&
        (r->lanes_valid >= LANES_VALID_FRAMES)) {

        /* If online then use the online lane values */
        if (r->online!=0) {
            lanes = r->lanes_online;
        }

        /* clear image */
        memset((void*)r->image_vehicles,
               '\0',r->image_width*r->image_height*3);

        /* try a few shadow thresholds */
#pragma omp parallel for
        for (layer = 0; layer < SHADOW_THRESHOLDS; layer++) {

            switch (layer) {
            case 0: {
                shadow_threshold_percent=50;
                break;
            }
            case 1: {
                shadow_threshold_percent=80;
                break;
            }
            case 2: {
                shadow_threshold_percent=70;
                break;
            }
            }

            /* detect shadows */
            vehicledetect_shadows(
                r->image_width, r->image_height, r->image_data,
                r->horizon, r->bonnet,
                lanes,
                r->reprojected_lookup,
                r->projected_width, r->projected_height,
                r->image_segmented, r->image_vehicles,
                layer, shadow_threshold_percent);

            /* detect shadow regions which appear rectangular
               and whose position is commensurate with a typical range of
               vehicle widths, assuming flat ground relative to the vehicle */
            r->no_of_possible_vehicles[layer] =
                vehicledetect_shadow_regions(
                    r->image_width, r->image_height,
                    r->image_segmented,
                    r->image_data,
                    r->image_vehicles,
                    r->ground_plane_width_mm,
                    r->vehicle_width_mm,
                    r->min_distance_mm,
                    r->max_distance_mm,
                    r->vehicle_max_range_ahead_mm,
                    r->reprojected_lookup,
                    r->projected_width, r->projected_height,
                    r->possible_vehicles[layer],
                    r->stack[layer],
                    layer,
                    r->horizon,
                    r->bonnet,
                    r->range_lookup);
        }

        /* consolidate the vehicle detection results
           into the first element */
        if (r->no_of_possible_vehicles[0]==0) {
            for (layer=1; layer<SHADOW_THRESHOLDS; layer++) {
                for (i=0; i<r->no_of_possible_vehicles[layer]; i++) {
                    for (j=0; j<POSSIBLE_VEHICLES_FIELDS; j++) {
                        r->possible_vehicles
                            [0]
                            [r->no_of_possible_vehicles[0]*
                             POSSIBLE_VEHICLES_FIELDS+j] =
                             r->possible_vehicles
                                 [layer]
                                 [i*POSSIBLE_VEHICLES_FIELDS+j];
                    }
                    r->no_of_possible_vehicles[0]++;
                    if (r->no_of_possible_vehicles[0] ==
                        MAX_POSSIBLE_VEHICLES) {
                        layer=9999;
                        break;
                    }
                }
            }
        }

        if (r->online!=0) {
            r->time_interval_mS = roadcv_time_elapsed_mS(r);
            if (r->time_interval_mS > 0) {
                /* is this observation an inlier? */
                online_validated =
                    vehicledetect_online_validation(
                        r->no_of_possible_vehicles[0],
                        r->possible_vehicles[0],
                        r->possible_vehicles_history,
                        r->range_tollerance_mm);

                /* update observations of the vehicle */
                vehicledetect_online_update(
                    r->no_of_possible_vehicles[0],
                    r->possible_vehicles[0],
                    r->possible_vehicles_history,
                    r->time_interval_mS);

                if (online_validated==0) {
                    r->no_of_possible_vehicles[0] = 0;
                    r->vehicle_current_range_ahead_mm = 0;
                }
                else {
                    /* update the current range estimate to
                       the vehicle ahead */
                    if (r->vehicle_current_range_ahead_mm>0) {
                        r->vehicle_current_range_ahead_mm =
                            (r->vehicle_current_range_ahead_mm*6+
                            r->possible_vehicles[0][
                                (online_validated-1)*
                                POSSIBLE_VEHICLES_FIELDS +
                                VEHICLES_FIELD_RANGE]*2)>>3;

                        /* store back within possible_vehicles array
                           so that the range estimate is shown when
                           drawing */
                        r->possible_vehicles[0][
                            (online_validated-1)*
                            POSSIBLE_VEHICLES_FIELDS +
                            VEHICLES_FIELD_RANGE]=
                            r->vehicle_current_range_ahead_mm;
                    }
                    else {
                        r->vehicle_current_range_ahead_mm =
                            r->possible_vehicles[0][
                                (online_validated-1)*
                                POSSIBLE_VEHICLES_FIELDS +
                                VEHICLES_FIELD_RANGE];
                    }

                    /* is the vehicle inside the stopping distance? */
                    if (r->stopping_distance_mm>0) {
                        if (r->vehicle_current_range_ahead_mm <
                            r->stopping_distance_mm) {
                            r->warnings |= WARNING_STOPPING_DISTANCE;
                        }
                    }
                }
            }
        }

        if (r->headless==0) {
            vehicledetect_show_vehicles(
                r->image_width, r->image_height, r->image_vehicles,
                r->no_of_possible_vehicles[0],
                r->possible_vehicles[0],
                255,255,255);
        }
    }
    else {
        r->no_of_possible_vehicles[0] = 0;
    }
}

void roadcv_update(struct rcv_data * r) {

    int i;

    /* clear any warnings */
    r->warnings=0;

    /* convert between BGR and RGB */
    if (r->BGR!=0) {
        roadcv_BGRtoRGB(
            r->image_width,
            r->image_height,
            r->image_data);
    }

    /* project from the original image onto the ground plane */
    road_project(
        r->image_width, r->image_height,
        r->image_data,
        r->projected_width, r->projected_height,
        r->projected_lookup, r->image_ground_plane);

    /* segment the road area */
    r->sample_width = r->projected_width*20/100;
    road_segment(
        r->projected_width,
        r->projected_height,
        r->sample_width,
        r->segmentation_threshold,
        r->image_ground_plane,
        r->image_ground_plane_buffer,
        r->image_segmented);

    /* locate road markings */
    roadcv_markings(r);

    /* detection of signs and vehicles can be done in parallel */
#pragma omp parallel for
    for (i=0; i<3; i++) {
        switch(i) {
            case 0: {
                roadcv_vehicles(r);
                break;
            }
            case 1: {
                roadcv_signs(r);
                break;
            }
            case 2: {
                roadcv_lights(r);
                break;
            }
        }
    }

    /* play audible warnings */
    if (r->enable_audio!=0) {
        warnings_audible(
            r->sounds_path, r->warnings, &(r->last_warning_time));
    }

    if (r->headless==0) {
        /* draw lanes on the original image */
        roadcv_draw_lanes(r);

        /* show warning icons */
        warnings_show_icons(
            r->warnings,
            r->image_width, r->image_height,
            r->image_data,
            &(r->current_warning_icon),
            &(r->last_warning_icon_time));

        /* convert between RGB and BGR */
        if (r->BGR!=0) {
            roadcv_BGRtoRGB(
                r->image_width,
                r->image_height,
                r->image_data);
        }
    }
}

void roadcv_debug(struct rcv_data * r, int update_colour_model) {

    int i;
    unsigned char * image_sign_colours;
    char str[32];

    if (r->detect_traffic_lights!=0) {
        sprintf((char*)str,"%s", "traffic_lights_filter.png");
        write_png_file(str, r->image_width, r->image_height,
            r->image_traffic_lights);

        if (r->no_of_traffic_lights[0]>0) {
            memcpy((void*)r->image_traffic_lights,(void*)r->image_data,
                r->image_width*r->image_height*3);
            trafficlights_show(
                r->image_width, r->image_height,
                r->image_traffic_lights,
                r->no_of_traffic_lights[0],
                r->traffic_lights[0], 0);
            sprintf((char*)str,"%s", "traffic_lights.png");
            write_png_file(str, r->image_width, r->image_height,
                r->image_traffic_lights);

            memcpy((void*)r->image_traffic_lights,(void*)r->image_data,
                r->image_width*r->image_height*3);
            trafficlights_show_search_region(
                r->image_width, r->image_height, r->image_traffic_lights,
                r->height_lookup, r->horizon,
                &(r->road_lines[8]));
            sprintf((char*)str,"%s", "traffic_lights_regions.png");
            write_png_file(str, r->image_width, r->image_height,
                r->image_traffic_lights);
        }
    }

    if (r->detect_vehicles!=0) {
        sprintf((char*)str,"%s", "shadows.png");
        write_png_file(
            str,
            r->image_width,
            r->image_height,
            r->image_vehicles);
        for (i=0; i<r->image_width*r->image_height*3; i++) {
            r->image_vehicles[i] = r->image_data[i];
        }
        if (r->no_of_possible_vehicles[0]>0) {
            vehicledetect_show_vehicles(
                r->image_width, r->image_height, r->image_vehicles,
                r->no_of_possible_vehicles[0],
                r->possible_vehicles[0],
                255,0,0);
            sprintf((char*)str,"%s", "vehicles.png");
            write_png_file(
                str,
                r->image_width,
                r->image_height,
                r->image_vehicles);
        }
    }

    if (r->vehicle_range_ahead_mm>0) {
        printf("Vehicle %d metres ahead\n",r->vehicle_range_ahead_mm/1000);
    }

    sprintf((char*)str,"%s", "original.png");
    write_png_file(str, r->image_width, r->image_height, r->image_data);
    sprintf((char*)str,"%s", "ground_plane.png");
    write_png_file(
        str,
        r->projected_width,
        r->projected_height,
        r->image_ground_plane);
    sprintf((char*)str,"%s", "segmented.png");
    write_png_file(
        str,
        r->projected_width,
        r->projected_height,
        r->image_segmented);
    sprintf((char*)str,"%s", "road_markings.png");
    write_png_file(
        str,
        r->projected_width,
        r->projected_height,
        r->image_road_markings);

    road_reproject(
        r->projected_width,
        r->projected_height,
        r->image_segmented,
        r->image_width, r->image_height,
        r->reprojected_lookup, r->image_data);
    sprintf((char*)str,"%s", "reprojected.png");
    write_png_file(str, r->image_width, r->image_height, r->image_data);

    if (r->detect_signs!=0) {
        sprintf((char*)str,"%s", "road_signs.png");
        write_png_file(
            str,
            r->image_width,
            r->image_height,
            r->image_road_signs);
    }

    if (update_colour_model!=0) {
        image_sign_colours = (unsigned char*)malloc(256*256*3);
        roadsigns_colour_models(256,256,image_sign_colours);
        sprintf((char*)str,"%s", "road_sign_colours.png");
        write_png_file(str, 256, 256, image_sign_colours);
        free(image_sign_colours);
    }
    if ((r->detect_signs != 0) &&
            (r->sign_region[0] > 0)) {
        roadsigns_show(
            r->image_width,
            r->image_data,
            r->shape_index,
            r->sign_region[0],
            r->sign_region[1],
            r->sign_region[2],
            r->sign_region[3]);
        sprintf((char*)str,"%s", "sign.png");
        write_png_file(str, r->image_width, r->image_height, r->image_data);

        if ((r->sign_index > -1) &&
                (r->shape_index > -1)) {
            printf("sign: %s\n", r->sign_text[r->shape_index][r->sign_index]);
        }
    }
}


