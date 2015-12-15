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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include "roadcv.h"

struct rcv_data rcv;

void show_help()
{
    printf("Options:\n\n");
    printf("    --elevation     ");
    printf("Elevation of the camera above the ground in millimetres.\n");
    printf("    --fov           ");
    printf("Field of view of the camera in degrees.\n");
    printf("    --tilt          ");
    printf("Tilt angle of the camera in degrees.  ");
    printf("Positive tilt is towards the ground.\n");
    printf("    --lanes         ");
    printf("Show detected lanes.\n");
    printf("    --vehicles      ");
    printf("Detect vehicles.\n");
    printf("    --signs         ");
    printf("Detect road signs.\n");
    printf("    --signsdata     ");
    printf("Load road signs database.\n");
    printf("-f  --filename      ");
    printf("Image to be loaded.  This must be in PNG format.\n");
    printf("    --debug         ");
    printf("Saves debugging images showing different ");
    printf("stanges of analysis.\n");
    printf("    --colourmodel   ");
    printf("Update road sign colour model.\n");
    printf("    --projectedwidth   ");
    printf("Width of the projected image\n");
    printf("    --projectedheight   ");
    printf("Height of the projected image\n");
}

int main(int argc, char* argv[]){
    int i,j,x,y,detect_signs=0,detect_vehicles=0,show_lanes=0;
    int detect_traffic_lights=0;
    int camera_height_mm=1000,fov=75,tilt=4,image_width=0,image_height=0;
    int update_sign_colour_model=0, debug=0;
    int projected_width=320/2;
    int projected_height=240;
    char * filename=0;
    char * signs_database=0;
    unsigned char * image_data=0;
    FILE * image_file;

    for (i=1;i<argc;i++) {
        /* show help */
        if ((strcmp(argv[i],"-h")==0) || (strcmp(argv[i],"--help")==0)) {
            show_help();
            return 0;
        }
        /* camera height above the ground */
        if (strcmp(argv[i],"--elevation")==0) {
            camera_height_mm = atoi(argv[i+1]);
            printf("Camera height %d mm\n", camera_height_mm);
            i++;
        }
        /* field of view in degrees */
        if (strcmp(argv[i],"--fov")==0) {
            fov = atoi(argv[i+1]);
            printf("Field of view %d degrees\n", fov);
            i++;
        }
        /* tilt degrees (positive is down) */
        if (strcmp(argv[i],"--tilt")==0) {
            tilt = atoi(argv[i+1]);
            printf("Tilt %d degrees\n", tilt);
            i++;
        }
        /* projected width */
        if (strcmp(argv[i],"--projectedwidth")==0) {
            projected_width = atoi(argv[i+1]);
            printf("Projected image width %d\n", tilt);
            i++;
        }
        /* projected height */
        if (strcmp(argv[i],"--projectedheight")==0) {
            projected_height = atoi(argv[i+1]);
            printf("Projected image height %d\n", tilt);
            i++;
        }
        /* show lanes */
        if (strcmp(argv[i],"--lanes")==0) {
            show_lanes = 1;
        }
        /* load the image */
        if ((strcmp(argv[i],"-f")==0) ||
            (strcmp(argv[i],"--filename")==0)) {
            filename = argv[i+1];
            if (!(image_file = fopen(filename, "r"))) {
                printf("File not found\n%s\n",filename);
                return 0;
            }
            else {
                fclose(image_file);
            }
            read_png_file(filename);
            image_width=width;
            image_height=height;
            image_data =
                (unsigned char*)malloc(image_width*image_height*3);
            j=0;
	        for (y=0; y<height; y++) {
  	            for (x=0; x<width; x++,j+=3) {
		            image_data[j] =
		                (unsigned char)(row_pointers[y][x*3]);
		            image_data[j+1] =
		                (unsigned char)(row_pointers[y][x*3+1]);
		            image_data[j+2] =
		                (unsigned char)(row_pointers[y][x*3+2]);
                }
            }
            i++;
        }
        /* debugging enabled */
        if (strcmp(argv[i],"--debug")==0) {
            debug=1;
        }
        /* detect vehicles */
        if (strcmp(argv[i],"--vehicles")==0) {
            detect_vehicles=1;
        }
        /* detect road signs */
        if (strcmp(argv[i],"--signs")==0) {
            detect_signs=1;
        }
        /* detect traffic lights */
        if (strcmp(argv[i],"--lights")==0) {
            detect_traffic_lights=1;
        }
        /* load road signs database */
        if (strcmp(argv[i],"--signsdata")==0) {
            signs_database = argv[i+1];
            printf("Signs database %s\n", signs_database);
            i++;
        }
        /* update road sign colour model */
        if ((strcmp(argv[i],"--colourmodel")==0) ||
            (strcmp(argv[i],"--colormodel")==0)) {
            update_sign_colour_model=1;
        }
    }

    if ((image_data!=0) && (image_width>0)) {

        roadcv_open(
            image_width,image_height,
            projected_width,projected_height,
            image_data,
            0,fov,tilt,camera_height_mm,
            detect_vehicles,detect_signs,
            detect_traffic_lights,0,
            NULL, signs_database,
            &rcv,0,0,show_lanes,0,0);
        assert(rcv.vehicle_width_mm==2000);

        printf("<?xml encoding=\"ISO-8859-1\"?>\n");
        printf("<Roadcv>\n");
        printf("  <Image>%s</Image>\n", filename);
        printf("  <Resolution>%d %d</Resolution>\n",
            image_width,image_height);

        roadcv_update(&rcv);

        if ((rcv.detect_vehicles!=0) &&
            (rcv.no_of_possible_vehicles[0]>0)) {
            printf("  <Vehicles>\n");
            for (i=0;i<rcv.no_of_possible_vehicles[0];i++) {
                printf("    <PossibleVehicle>\n");
                printf("      <ImageRegion>%d %d %d %d</ImageRegion>\n",
                    rcv.possible_vehicles[0][i*POSSIBLE_VEHICLES_FIELDS],
                    rcv.possible_vehicles[0][i*POSSIBLE_VEHICLES_FIELDS+1],
                    rcv.possible_vehicles[0][i*POSSIBLE_VEHICLES_FIELDS+2],
                    rcv.possible_vehicles[0][i*POSSIBLE_VEHICLES_FIELDS+3]);
                printf("      <RangeMillimetres>%d</RangeMillimetres>\n",
                    rcv.possible_vehicles[0][i*POSSIBLE_VEHICLES_FIELDS+4]);
                printf("    </PossibleVehicle>\n");

            }
            printf("  </Vehicles>\n");
        }

        if (rcv.road_lines[8]!=0) {
            printf("  <LaneMarkings>\n");
            printf("    <Left>\n");
            printf("        <LinePixels>%d %d %d %d</LinePixels>\n",
                rcv.road_lines[8],rcv.road_lines[9],
                rcv.road_lines[10],rcv.road_lines[11]);
            printf("        <LineMillimetres>");
            printf("%d %d %d %d</LineMillimetres>\n",
                (rcv.road_lines[0]-(rcv.projected_width/2))*
                rcv.ground_plane_width_mm/rcv.projected_width,
                rcv.min_distance_mm +
                (rcv.projected_height - rcv.road_lines[1]) *
                rcv.ground_plane_width_mm/rcv.projected_width,
                (rcv.road_lines[2]-(rcv.projected_width/2)) *
                rcv.ground_plane_width_mm/rcv.projected_width,
                rcv.min_distance_mm +
                (rcv.projected_height-rcv.road_lines[3]) *
                rcv.ground_plane_width_mm/rcv.projected_width);
            printf("    </Left>\n");
            printf("    <Right>\n");
            printf("        <LinePixels>%d %d %d %d</LinePixels>\n",
                rcv.road_lines[12],rcv.road_lines[13],
                rcv.road_lines[14],rcv.road_lines[15]);
            printf("        <LineMillimetres>");
            printf("%d %d %d %d</LineMillimetres>\n",
                (rcv.road_lines[4]-
                (rcv.projected_width/2))*
                rcv.ground_plane_width_mm/rcv.projected_width,
                rcv.min_distance_mm +
                (rcv.projected_height-rcv.road_lines[5])*
                rcv.ground_plane_width_mm/rcv.projected_width,
                (rcv.road_lines[6]-(rcv.projected_width/2))*
                rcv.ground_plane_width_mm/rcv.projected_width,
                rcv.min_distance_mm +
                (rcv.projected_height-rcv.road_lines[7])*
                rcv.ground_plane_width_mm/rcv.projected_width);
            printf("    </Right>\n");
            printf("  </LaneMarkings>\n");
        }
        if (rcv.no_of_traffic_lights[0]>0) {
            printf("  <TrafficLights>\n");
            for (i=0;i<rcv.no_of_traffic_lights[0];i++) {
                printf("    <PossibleTrafficLight>\n");
                printf("      <ImageRegion>%d %d %d %d</ImageRegion>\n",
                    rcv.traffic_lights[0][i*5],
                    rcv.traffic_lights[0][i*5+1],
                    rcv.traffic_lights[0][i*5+2],
                    rcv.traffic_lights[0][i*5+3]);
                printf("      <State>");
                switch(rcv.traffic_lights[0][i*5+4]) {
                    case TRAFFICLIGHT_STATE_RED: {
                        printf("Red");
                        break;
                    }
                    case TRAFFICLIGHT_STATE_AMBER: {
                        printf("Amber");
                        break;
                    }
                    case TRAFFICLIGHT_STATE_GREEN: {
                        printf("Green");
                        break;
                    }
                }
                printf("</State>\n");
                printf("    </PossibleTrafficLight>\n");
            }
            printf("  </TrafficLights>\n");
        }

        printf("</Roadcv>\n");

        if (debug!=0) roadcv_debug(&rcv, update_sign_colour_model);
        roadcv_close(&rcv);

    }
    else {
        printf("No left or right image was missing\n");
    }
	return(1);
}

