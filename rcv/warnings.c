/*
 warnings of various kinds
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

#include "warnings.h"

int warnings_play_sound(
    char * sounds_path,
    const char * sound_filename)
{
    char command_str[256];
    FILE * file;

    if ((file = fopen(sound_filename, "r"))) {
        fclose(file);
        if (sounds_path!=NULL) {
            sprintf((char*)command_str,"%s %s/%s",
                SOUND_COMMAND, sounds_path, sound_filename);
            return system(command_str);
        }
        else {
            printf("No path specified to sound files with --sounds\n");
            return -1;
        }
    }
    else {
        printf("Sound file %s not found\n", sound_filename);
        return -1;
    }
}

void warnings_audible(
    char * sounds_path,
    int flags,
    long int * last_warning_time)
{
    long int elapsed_mS,curr_t;
    struct timeval t;

    if (flags!=0) {
        gettimeofday(&t, NULL);
        curr_t = (long int)(t.tv_usec + 1000000 * t.tv_sec);
        elapsed_mS = ABS((curr_t - *last_warning_time)/1000);

        if (elapsed_mS > WARNING_TIME_INTERVAL_MS) {
            *last_warning_time = curr_t;

            if (flags & WARNING_STOPPING_DISTANCE) {
                warnings_play_sound(
                    sounds_path, WARNING_SOUND_STOPPING_DISTANCE);
                return;
            }
            if (flags & WARNING_STOP_LIGHT) {
                warnings_play_sound(sounds_path, WARNING_SOUND_STOP_LIGHT);
                return;
            }
            if (flags & WARNING_LANE_CROSSING) {
                warnings_play_sound(
                    sounds_path, WARNING_SOUND_LANE_CROSSING);
                return;
            }
            if (flags & WARNING_SPEED_LIMIT) {
                warnings_play_sound(sounds_path, WARNING_SOUND_SPEED_LIMIT);
                return;
            }
            if (flags & WARNING_GO) {
                warnings_play_sound(sounds_path, WARNING_SOUND_GO);
                return;
            }
        }
    }
}

void warnings_draw_circle_icon(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int r,
    int g,
    int b)
{
    int x,y,cx,cy,radius,radius2,n,dx,dy;

    cx = image_width>>1;
    cy = MULDIVINT32(image_height,10,100);
    radius = MULDIVINT32(image_height,5,100);
    radius2=radius*radius;
    for (y = cy-radius; y < cy+radius; y++) {
        dy = y - cy;
        n = (y*image_width+cx-radius)*3;
        for (x = cx-radius; x < cx+radius; x++, n+=3) {
            dx = x - cx;
            if (dx*dx + dy*dy < radius2) {
                image_data[n]=r;
                image_data[n+1]=g;
                image_data[n+2]=b;
            }
        }
    }
}

void warnings_show_icons(
    int flags,
    int image_width,
    int image_height,
    unsigned char * image_data,
    int * current_warning_icon,
    long int * last_warning_icon_time)
{
    long int elapsed_mS,curr_t;
    struct timeval t;

    gettimeofday(&t, NULL);
    curr_t = (long int)(t.tv_usec + 1000000 * t.tv_sec);
    elapsed_mS = ABS((curr_t - *last_warning_icon_time)/1000);

    if (elapsed_mS > WARNING_TIME_INTERVAL_MS) {
        *current_warning_icon = 0;

        if (flags & WARNING_GO) {
            *current_warning_icon = WARNING_GO;
            *last_warning_icon_time = curr_t;
        }
        if (flags & WARNING_STOP_LIGHT) {
            *current_warning_icon = WARNING_STOP_LIGHT;
            *last_warning_icon_time = curr_t;
        }
    }

    switch(*current_warning_icon) {
        case WARNING_GO: {
            warnings_draw_circle_icon(
                image_width, image_height, image_data, 0,220,0);
            break;
        }
        case WARNING_STOP_LIGHT: {
            warnings_draw_circle_icon(
                image_width, image_height, image_data, 220,0,0);
            break;
        }
    }
}

