/*
 detection of road signs
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

#include "roadsigns.h"

void roadsigns_colour_model(
    int no_of_examples,
    unsigned char * examples,
    int image_width,
    int image_height,
    unsigned char * image_sign_colours,
    unsigned char * model)
{
    int i,j,k,r,g,b,r2,g2,b2,u,v,n;
    int u1,v1,u2,v2,x,y;

    for (i=0;i<no_of_examples;i++) {
        r = examples[i*3];
        g = examples[i*3+1];
        b = examples[i*3+2];
        u1 = MIN( ABS(r*-1214 + g*-2384 + b*3598 + 4096 + 1048576)
            >> 13, 240);
        v1 = MIN( ABS(r*3598 + g*-3013 + b*-585 + 4096 + 1048576)
            >> 13, 240);
        for (j=i+1;j<no_of_examples;j++) {
            r2 = examples[j*3];
            g2 = examples[j*3+1];
            b2 = examples[j*3+2];
            u2 = MIN(
                ABS(r2*-1214 + g2*-2384 + b2*3598 + 4096 + 1048576)
                >> 13, 240);
            v2 = MIN( ABS(r2*3598 + g2*-3013 + b2*-585 + 4096 + 1048576)
                >> 13, 240);
            for (k=0;k<50;k++) {
                u = u1 + (k*(u2-u1)/50);
                v = v1 + (k*(v2-v1)/50);
                for (x=u-2;x<=u+2;x++) {
                    for (y=v-2;y<=v+2;y++) {
                        n = (x*256+y)*3;
                        image_sign_colours[n] = r;
                        image_sign_colours[n+1] = g;
                        image_sign_colours[n+2] = b;
                        n = (x>>3)*32+(y>>3);
                        model[n>>3] |= (1<<(n-((n>>3)<<3)));
                    }
                }
            }
        }
    }
}

void roadsigns_colour_models(
    int image_width,
    int image_height,
    unsigned char * image_sign_colours)
{
    unsigned char model[1024];
    int i,red_examples = 16;
    unsigned char red[] = {
        184,42,24,
        168,57,14,
        141,55,0,
        131,71,0,
        126,71,48,
        227,81,65,
        231,126,110,
        183,22,25,
        117,54,61,
        200,44,50,
        139,34,18,
        200,44,50,
        65,43,44,
        43,32,24,
        70,52,46,
        45,28,26
    };

/*
    int blue_examples = 11;
    unsigned char blue[] = {
        38,48,72,
        38,48,68,
        42,47,65,
        18,75,132,
        49,139,224,
        69,95,127,
        60,133,212,
        100,131,191,
        59,134,177,
        50,147,204,
        83,101,124
    };

    int green_examples = 11;
    unsigned char green[] = {
        30,68,51,
        30,67,53,
        51,150,109,
        32,171,102,
        17,133,21,
        21,153,73,
        108,151,100,
        90,144,61,
        52,142,105,
        57,103,51,
        84,147,95
    };

    int yellow_examples = 9;
    unsigned char yellow[] = {
        234,206,14,
        189,149,33,
        234,174,16,
        181,132,14,
        108,79,50,
        119,87,41,
        234,235,15,
        232,206,18,
        233,220,15
    };
*/
    for (i=0;i<image_width*image_height*3;i++) image_sign_colours[i]=255;
    for (i=0;i<1024;i++) model[i]=0;

    roadsigns_colour_model(
        red_examples,
        red, image_width,
        image_height,
        image_sign_colours,
        model);
    /*roadsigns_colour_model(
        blue_examples,
        blue,
        image_width,
        image_height,
        image_sign_colours,
        model);*/
    /*roadsigns_colour_model(
        green_examples,
        green,
        image_width,
        image_height,
        image_sign_colours,
        model);*/
    /*roadsigns_colour_model(
        yellow_examples,
        yellow,
        image_width,
        image_height,
        image_sign_colours,
        model);*/

    printf("    unsigned char road_sign_colour_model[] = {\n        ");
    for (i=0;i<1024;i++) {
        printf("%d", model[i]);
        if (i<1023) {
            printf(",");
            if (i%16==15) printf("\n        ");
        }
    }
    printf("\n    };\n");
}


void roadsigns_recognise(
    int image_width,
    int image_height,
    unsigned char * image_data,
    unsigned char * image_road_signs,
    int tx,
    int ty,
    int bx,
    int by,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int * shape_index,
    int ** sign_matches)
{
    int n,left_x,right_x,x,y,symmetry=0;
    int av,hits;
    *shape_index = -1;

    /* check symmetry */
    for (y=ty;y<=by;y++) {
        n = (y*image_width+tx)*3;
        for (left_x=tx;left_x<=bx;left_x++,n+=3) {
            if (image_road_signs[n]>0) break;
        }
        n = (y*image_width+bx)*3;
        for (right_x=bx;right_x>=tx;right_x--,n-=3) {
            if (image_road_signs[n]>0) break;
        }
        symmetry += ABS((left_x-tx)-(bx-right_x));
    }
    symmetry = 100 - (symmetry*200/((by-ty)*(bx-tx)));

    if (symmetry > minimum_symmetry) {

        /* mean reflectance */
        av=0;
        hits=0;
        for (y=ty;y<=by;y++) {
            n = (y*image_width+tx)*3;
            for (left_x=tx;left_x<=bx;left_x++,n+=3) {
                if (image_road_signs[n]>0) break;
            }
            n = (y*image_width+bx)*3;
            for (right_x=bx;right_x>=tx;right_x--,n-=3) {
                if (image_road_signs[n]>0) break;
            }
            n = (y*image_width+left_x)*3;
            for (x=left_x;x<=right_x;x++,n+=3) {
                av += image_road_signs[n];
                hits++;
            }
        }

        if (hits>0) {
            /* show */
            av /= hits;
            av = av * 110/100;
            for (y=ty;y<=by;y++) {
                n = (y*image_width+tx)*3;
                for (left_x=tx;left_x<=bx;left_x++,n+=3) {
                    if (image_road_signs[n]>0) break;
                }
                n = (y*image_width+bx)*3;
                for (right_x=bx;right_x>=tx;right_x--,n-=3) {
                    if (image_road_signs[n]>0) break;
                }
                n = (y*image_width+left_x)*3;
                for (x=left_x;x<=right_x;x++,n+=3) {
                    if (image_road_signs[n]==0) {
                        if (image_data[n]>av) {
                            image_road_signs[n] = 255;
                            image_road_signs[n+1] = 255;
                            image_road_signs[n+2] = 255;
                        }
                        else {
                            image_road_signs[n] = 1;
                            image_road_signs[n+1] = 1;
                            image_road_signs[n+2] = 1;
                        }
                    }
                }
            }

            /* find best shape */
            *shape_index = roadsigns_identify_shape(
                image_width, image_height, image_road_signs,
                tx, ty, bx, by, sign_shapes);
            if (*shape_index > -1) {

                roadsigns_identify_sign(
                    image_width, image_height, image_road_signs,
                    tx, ty, bx, by,
                    *shape_index, no_of_signs,
                    descriptors, eigen_descriptors, sign_matches);

            }
        }
    }
}

int roadsigns_regions_join(
    unsigned short * regions,
    int no_of_regions,
    int min_overlap)
{
    int i,j,tx0,ty0,bx0,by0,tx1,ty1,bx1,by1,ctr=0;
    int tx,ty,bx,by;
    for (i=0;i<no_of_regions;i++) {
        tx0 = regions[i*4];
        if (tx0>0) {
            ty0 = regions[i*4+1];
            bx0 = regions[i*4+2];
            by0 = regions[i*4+3];
            for (j=i+1;j<no_of_regions;j++) {
                tx1 = regions[j*4];
                if (tx1>0) {
                    ty1 = regions[j*4+1];
                    bx1 = regions[j*4+2];
                    by1 = regions[j*4+3];

                    if (tx0 < bx1 && tx1 < bx0 &&
                        ty0 < by1 && ty1 < by0) {

                        if (tx0>tx1) tx = tx0; else tx = tx1;
                        if (bx0>bx1) bx = bx0; else bx = bx1;
                        if (ty0>ty1) ty = ty0; else ty = ty1;
                        if (by0>by1) by = by0; else by = by1;
                        if ((tx - bx) * (ty - by) > min_overlap) {
                            if (tx1 < tx0) tx0 = tx1;
                            if (bx1 > bx0) bx0 = bx1;
                            if (ty1 < ty0) ty0 = ty1;
                            if (by1 > by0) by0 = by1;
                            regions[j*4]=0;
                        }
                    }
                }
            }
            regions[i*4] = tx0;
            regions[i*4+1] = ty0;
            regions[i*4+2] = bx0;
            regions[i*4+3] = by0;
            ctr++;
        }
    }

    if (ctr!=no_of_regions) {
        j=0;
        for (i=0;i<no_of_regions;i++) {
            if (regions[i*4]!=0) {
                if (i!=j) {
                    regions[j*4] = regions[i*4];
                    regions[j*4+1] = regions[i*4+1];
                    regions[j*4+2] = regions[i*4+2];
                    regions[j*4+3] = regions[i*4+3];
                }
                j++;
            }
        }
    }

    return ctr;
}

void roadsigns_analyse_regions(
    int image_width,
    int image_height,
    int horizon,
    unsigned char * image_data,
    unsigned char * image_road_signs,
    int r,
    int g,
    int b,
    int filled_r,
    int filled_g,
    int filled_b,
    int minimum_sign_width,
    int maximum_sign_width,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int * shape_index,
    int * sign_index,
    int ** sign_matches,
    int * tx,
    int * ty,
    int * bx,
    int * by,
    unsigned short * stack,
    unsigned short * regions)
{
    int i,j,x,y,tx2=0,ty2=0,bx2=0,by2=0;
    int n=0, region_width, region_height, aspect;
    int no_of_regions=0, max_sign_matches;
    int stack_size = (image_width>>2)*(image_height>>2)*2;
    int temp_shape_index=0;

    *shape_index = -1;

    for (y=0;y<horizon;y++) {
        for (x=0;x<image_width;x++, n+=3) {
            if ((image_road_signs[n]==r) &&
                (image_road_signs[n+1]==g) &&
                (image_road_signs[n+2]==b)) {
                floodfill_fill(
                    x, y, image_width, image_height,
                    stack, stack_size, image_road_signs,
                    r, g, b,
                    filled_r, filled_g, filled_b,
                    &tx2, &ty2, &bx2, &by2);
                region_width = bx2 - tx2;
                if ((region_width > minimum_sign_width) &&
                    (region_width < maximum_sign_width)) {
                    region_height = by2 - ty2;
                    aspect = region_height * 100 / region_width;
                    if ((aspect>70) && (aspect<130)) {

                        regions[no_of_regions*4] = tx2;
                        regions[no_of_regions*4+1] = ty2;
                        regions[no_of_regions*4+2] = bx2;
                        regions[no_of_regions*4+3] = by2;
                        no_of_regions++;
                        if (no_of_regions==ROADSIGNS_MAX_REGIONS) {
                            y=horizon;
                            break;
                        }

                    }
                }
            }
        }
    }

    if (no_of_regions>0) {
        no_of_regions = roadsigns_regions_join(regions, no_of_regions,5);

        max_sign_matches=0;
        for (i=0;i<no_of_regions;i++) {
            if (i==0) {
                *tx = regions[i*4];
                *ty = regions[i*4+1];
                *bx = regions[i*4+2];
                *by = regions[i*4+3];
            }

            roadsigns_recognise(
                image_width, image_height, image_data, image_road_signs,
                regions[i*4],regions[i*4+1],regions[i*4+2],regions[i*4+3],
                minimum_symmetry,
                no_of_signs, sign_shapes,
                descriptors, eigen_descriptors,
                &temp_shape_index,
                sign_matches);

            if (temp_shape_index > -1) {
                max_sign_matches=0;
                for (j=0;j<no_of_signs[temp_shape_index];j++) {
                    if (sign_matches[temp_shape_index][j]>
                        max_sign_matches) {
                        max_sign_matches =
                            sign_matches[temp_shape_index][j];
                        *sign_index = j;
                        *shape_index = temp_shape_index;
                        *tx = regions[i*4];
                        *ty = regions[i*4+1];
                        *bx = regions[i*4+2];
                        *by = regions[i*4+3];
                    }
                }
            }
        }
    }
}

void roadsigns_detect(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int horizon,
    int minimum_sign_width,
    int maximum_sign_width,
    int minimum_symmetry,
    int * no_of_signs,
    unsigned char * sign_shapes,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    unsigned char * model,
    unsigned char * image_road_signs,
    int * shape_index,
    int * sign_index,
    int ** sign_matches,
    int * tx,
    int * ty,
    int * bx,
    int * by,
    unsigned short * stack,
    unsigned short * regions)
{
    int x,y,U,V,i,n=0,n2,r,g,b,bit;

    *sign_index=-1;

    for (n=0;n<image_width*image_height*3;n++) image_road_signs[n]=0;
    n=0;
    for (y=0;y<horizon;y++) {
        for (x=0;x<image_width;x++,n+=3) {
            r = image_data[n];
            g = image_data[n+1];
            b = image_data[n+2];
            U = (MIN(
                ABS(r*-1214 + g*-2384 + b*3598 + 4096 + 1048576)
                >> 13, 240)) >> 3;
            V = (MIN(
                ABS(r*3598 + g*-3013 + b*-585 + 4096 + 1048576)
                >> 13, 240)) >> 3;
            n2 = U*32+V;
            bit = model[n2>>3] & (1<<(n2-((n2>>3)<<3)));
            if (bit) {
                if (ABS(r - b) > 5) {
                    if ((r > b+30) && (ABS(g-b)<=ABS(r-b)>>1)) {
                        /*red*/
                        r = 255;
                        g = 0;
                        b = 0;
                    }
                    else {
                        if ((g>b) && (ABS(g-b) < 30)) {
                            r = g = b = 0;
                        }
                        else {
                            if ((b > r) && (b > g)) {
                                /*blue*/
                                r=g=0;
                                b=255;
                            }
                            else {
                                if ((g>r+30) && (g>b+30)) {
                                    /*green*/
                                    r=b=0;
                                    g=255;
                                }
                                else {
                                    if ((r+g)>>1>b+80) {
                                        /*yellow*/
                                        r=g=0;
                                        b=0;
                                    }
                                    else {
                                        r=g=b=0;
                                    }
                                }
                            }
                        }
                    }
                    image_road_signs[n] = r;
                    image_road_signs[n+1] = g;
                    image_road_signs[n+2] = b;
                }
                else {
                    if ((g>r+30) && (g>b+30)) {
                        r=b=0;
                        g=255;
                        image_road_signs[n] = r;
                        image_road_signs[n+1] = g;
                        image_road_signs[n+2] = b;
                    }
                }
            }
        }
    }

    roadsigns_analyse_regions(
        image_width, image_height,
        horizon,
        image_data,
        image_road_signs,
        255,0,0,
        254,0,0,
        minimum_sign_width,
        maximum_sign_width,
        minimum_symmetry,
        no_of_signs,
        sign_shapes,
        descriptors,
        eigen_descriptors,
        shape_index,
        sign_index,
        sign_matches,
        tx,ty,bx,by,
        stack,
        regions);

    /* matches decay */
    for (n=0;n<ROADSIGNS_SHAPES;n++) {
        for (i=0;i<no_of_signs[n];i++) {
            sign_matches[n][i] = MULDIVINT32(sign_matches[n][i],80,100);
        }
    }
}

void roadsigns_create_shapes(
    unsigned char * sign_shapes)
{
    int x,y,n,max,cx,dx,dy;

    max = (ROADSIGNS_SHAPE_TEMPLATE_SIZE*
           ROADSIGNS_SHAPE_TEMPLATE_SIZE*
           ROADSIGNS_SHAPES/8)+1;
    for (n=0;n<max;n++) sign_shapes[n]=0;
    cx = ROADSIGNS_SHAPE_TEMPLATE_SIZE/2;
    n=0;

    /* triangle */
    for (y=0;y<ROADSIGNS_SHAPE_TEMPLATE_SIZE;y++) {
        for (x=0;x<ROADSIGNS_SHAPE_TEMPLATE_SIZE;x++,n++) {
            if (y > ABS((x-cx)*ROADSIGNS_SHAPE_TEMPLATE_SIZE/cx)) {
                sign_shapes[n>>3] |= 1<<(n%8);
            }
        }
    }

    /* circle */
    for (y=0;y<ROADSIGNS_SHAPE_TEMPLATE_SIZE;y++) {
        dy = y - cx;
        for (x=0;x<ROADSIGNS_SHAPE_TEMPLATE_SIZE;x++,n++) {
            dx = x - cx;
            if (dx*dx + dy*dy < cx*cx) {
                sign_shapes[n>>3] |= 1<<(n%8);
            }
        }
    }

    /* triangle inverted */
    for (y=0;y<ROADSIGNS_SHAPE_TEMPLATE_SIZE;y++) {
        for (x=0;x<ROADSIGNS_SHAPE_TEMPLATE_SIZE;x++,n++) {
            if (y < ROADSIGNS_SHAPE_TEMPLATE_SIZE-1-
                    ABS((x-cx)*ROADSIGNS_SHAPE_TEMPLATE_SIZE/cx)) {
                sign_shapes[n>>3] |= 1<<(n%8);
            }
        }
    }

    /* square */
    for (y=0;y<ROADSIGNS_SHAPE_TEMPLATE_SIZE;y++) {
        for (x=0;x<ROADSIGNS_SHAPE_TEMPLATE_SIZE;x++,n++) {
            sign_shapes[n>>3] |= 1<<(n%8);
        }
    }
}

int roadsigns_identify_shape(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned char * sign_shapes)
{
    int shape=-1;
    int x,y,n,n2,n3,xx,yy,w,h,sh,tsize,state,max=0;
    int shape_hits[ROADSIGNS_SHAPES];

    for (sh=0;sh<ROADSIGNS_SHAPES;sh++) shape_hits[sh]=0;

    tsize = ROADSIGNS_SHAPE_TEMPLATE_SIZE*ROADSIGNS_SHAPE_TEMPLATE_SIZE;
    w = bx-tx;
    h = by-ty;
    for (y=ty;y<by;y++) {
        yy = MULDIVINT32((y-ty),ROADSIGNS_SHAPE_TEMPLATE_SIZE,h);
        n = (y*image_width+tx)*3;
        for (x=tx;x<bx;x++,n+=3) {
            xx = MULDIVINT32((x-tx),ROADSIGNS_SHAPE_TEMPLATE_SIZE,w);
            n2 = yy*ROADSIGNS_SHAPE_TEMPLATE_SIZE+xx;
            if (image_data[n] + image_data[n+1] == 0) {
                state=0;
            }
            else {
                state=1;
            }
            for (sh=0;sh<ROADSIGNS_SHAPES;sh++) {
                n3 = sh*tsize + n2;
                if (sign_shapes[n3>>3] & 1<<(n3%8)) {
                    if (state==1) {
                        shape_hits[sh]++;
                    }
                    else {
                        shape_hits[sh]--;
                    }
                }
                else {
                    if (state==0) {
                        shape_hits[sh]++;
                    }
                    else {
                        shape_hits[sh]--;
                    }
                }
            }
        }
    }

    for (sh=0;sh < ROADSIGNS_SHAPES;sh++) {
        if (shape_hits[sh] > max) {
            max = shape_hits[sh];
            shape = sh;
        }
    }

    return shape;
}

void roadsigns_vertices(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int tx,
    int ty,
    int bx,
    int by,
    int * vertices)
{
    int x,y,n;

    /* locate left side */
    for (x=tx;x<=bx;x++) {
        for (y=by;y>=ty;y--) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                vertices[0] = x;
                vertices[1] = y;
                x = bx+1;
                break;
            }
        }
    }
    for (x=tx;x<=bx;x++) {
        for (y=ty;y<=by;y++) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                if (x == vertices[0]) {
                    vertices[1] = (vertices[1]+y)>>1;
                }
                x = bx+1;
                break;
            }
        }
    }

    /* locate right side */
    for (x=bx;x>=tx;x--) {
        for (y=by;y>=ty;y--) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                vertices[2] = x;
                vertices[3] = y;
                x = tx-1;
                break;
            }
        }
    }
    for (x=bx;x>=tx;x--) {
        for (y=ty;y<=by;y++) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                if (x == vertices[2]) {
                    vertices[3] = (vertices[3]+y)>>1;
                }
                x = tx-1;
                break;
            }
        }
    }

    /* locate top */
    for (y=ty;y<=by;y++) {
        for (x=tx;x<=bx;x++) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                vertices[4] = x;
                vertices[5] = y;
                y = by+1;
                break;
            }
        }
    }
    for (y=ty;y<=by;y++) {
        for (x=bx;x>=tx;x--) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                if (y == vertices[5]) {
                    vertices[4] = (vertices[4]+x)>>1;
                }
                y = by+1;
                break;
            }
        }
    }

    /* locate bottom */
    for (y=by;y>=ty;y--) {
        for (x=tx;x<=bx;x++) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                vertices[6] = x;
                vertices[7] = y;
                y = ty-1;
                break;
            }
        }
    }
    for (y=by;y>=ty;y--) {
        for (x=bx;x>=tx;x--) {
            n = (y*image_width+x)*3;
            if ((image_data[n]>200) &&
                (image_data[n+1]>200) &&
                (image_data[n+2]>200)) {
                if (y == vertices[7]) {
                    vertices[6] = (vertices[6]+x)>>1;
                }
                y = ty-1;
                break;
            }
        }
    }

}

void roadsigns_descriptor(
    int image_width,
    int image_height,
    unsigned char * image_data,
    int shape_index,
    int tx,
    int ty,
    int bx,
    int by,
    unsigned char * descriptor)
{
    int x,y,x0,x1,y0,y1,i,n,left_x,right_x,white,black,red;
    int top_y, bottom_y, half, cx=0;
    int state,prev_state,edges;
    int vertices[8];

    roadsigns_vertices(
        image_width,
        image_height,
        image_data,
        tx, ty, bx, by,
        vertices);

    /* horizontal scan */
    y0 = ty;
    y1 = by;
    for (i=0;i<ROADSIGNS_DESCRIPTOR_SIZE;i++) {
        y = y0 + MULDIVINT32(i,(y1-y0),ROADSIGNS_DESCRIPTOR_SIZE);
        n = (y*image_width+tx)*3;
        for (left_x=tx;left_x<=bx;left_x++,n+=3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]<100) &&
                (image_data[n+2]<100)) {
                break;
            }
        }
        n = (y*image_width+bx)*3;
        for (right_x=bx;right_x>=tx;right_x--,n-=3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]<100) &&
                (image_data[n+2]<100)) {
                break;
            }
        }

        white=0;
        black=0;
        red=0;
        state=-1;
        prev_state=-1;
        edges=0;
        n = (y*image_width+left_x)*3;
        for (x=left_x;x<=right_x;x++,n+=3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]>200)) {
                white++;
                state=1;
            }
            else {
                if ((image_data[n]<100) &&
                    (image_data[n+1]<100) &&
                    (image_data[n+2]<100)) {
                    black++;
                    state=2;
                }
                else {
                    if ((image_data[n]>200) &&
                        (image_data[n+1]<100) &&
                        (image_data[n+2]<100)) {
                        red++;
                        state=3;
                    }
                }
            }
            if (state!=prev_state) edges++;
            prev_state=state;
        }
        if (white+black>0) {
            descriptor[i] =
                (unsigned char)MULDIVINT32(black,255,(black+white));
        }
        else {
            descriptor[i] = 0;
        }
        if (white+black+red>0) {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE] =
                (unsigned char)MULDIVINT32(red,255,(black+white+red));
        }
        else {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE] = 0;
        }
        descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*2]=
            (unsigned char)(edges*10);
    }

    /* vertical scan */
    cx = tx + ((bx-tx)>>1);
    x0 = tx;
    x1 = bx;
    if (shape_index == ROADSIGNS_SHAPE_TRIANGLE) {
        if ((vertices[4]>x0) && (vertices[4] < x1)) cx = vertices[4];
    }
    else {
        if (shape_index == ROADSIGNS_SHAPE_TRIANGLE_INVERTED) {
            if ((vertices[6]>x0) && (vertices[6] < x1)) cx = vertices[6];
        }
    }
    half = ROADSIGNS_DESCRIPTOR_SIZE>>1;
    for (i=0;i<ROADSIGNS_DESCRIPTOR_SIZE;i++) {
        if (i<half) {
            x = x0 + MULDIVINT32(i,(cx-x0),half);
        }
        else {
            x = cx + MULDIVINT32(i-half,(x1-cx),half);
        }
        n = (y0*image_width+x)*3;
        for (top_y=y0;top_y<=y1;top_y++,n+=image_width*3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]<100) &&
                (image_data[n+2]<100)) {
                break;
            }
        }
        n = (y1*image_width+x)*3;
        for (bottom_y=y1;bottom_y>=y0;bottom_y--,n-=image_width*3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]<100) &&
                (image_data[n+2]<100)) {
                break;
            }
        }

        white=0;
        black=0;
        red=0;
        state=-1;
        prev_state=-1;
        edges=0;
        n = (top_y*image_width+x)*3;
        for (y=top_y;y<=bottom_y;y++,n+=image_width*3) {
            if ((image_data[n]>200) &&
                (image_data[n+1]>200)) {
                white++;
                state=1;
            }
            else {
                if ((image_data[n]<100) &&
                    (image_data[n+1]<100) &&
                    (image_data[n+2]<100)) {
                    black++;
                    state=2;
                }
                else {
                    if ((image_data[n]>200) &&
                        (image_data[n+1]<100) &&
                        (image_data[n+2]<100)) {
                        red++;
                        state=3;
                    }
                }
            }
            if (state!=prev_state) edges++;
            prev_state=state;
        }
        if (white+black>0) {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*3] =
                (unsigned char)MULDIVINT32(black,255,(black+white));
        }
        else {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*3] = 0;
        }
        if (white+black+red>0) {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*4] =
                (unsigned char)MULDIVINT32(red,255,(black+white+red));
        }
        else {
            descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*4] = 0;
        }
        descriptor[i+ROADSIGNS_DESCRIPTOR_SIZE*5] =
            (unsigned char)(edges*10);
    }
    /*
    printf("V  ");
    for (i=0;i<ROADSIGNS_DESCRIPTOR_SIZE*2;i++) {
        printf("%d ", (int)descriptor[i]);
    }
    printf("\nH  ");
    for (i=ROADSIGNS_DESCRIPTOR_SIZE*2;i<ROADSIGNS_DESCRIPTOR_SIZE*4;i++) {
        printf("%d ", (int)descriptor[i]);
    }
    printf("\n\n");
    */
}

void roadsigns_update_eigen_descriptors(
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors)
{
    int sh,i,j;
    int eigen[ROADSIGNS_DESCRIPTOR_SIZE*ROADSIGNS_DESCRIPTOR_ENTRIES];

    for (sh=0;sh<ROADSIGNS_SHAPES;sh++) {
        if (no_of_signs[sh]>0) {
            for (j=0;
                 j<ROADSIGNS_DESCRIPTOR_SIZE*ROADSIGNS_DESCRIPTOR_ENTRIES;
                 j++) {
                eigen[j]=0;
            }
            for (i=0;i<no_of_signs[sh];i++) {
                for (j=0;
                     j<ROADSIGNS_DESCRIPTOR_SIZE*
                       ROADSIGNS_DESCRIPTOR_ENTRIES;
                     j++) {
                    eigen[j] +=
                        descriptors[sh][ROADSIGNS_DESCRIPTOR_SIZE*
                            ROADSIGNS_DESCRIPTOR_ENTRIES*i + j];
                }
            }
            for (j = 0;
                 j < ROADSIGNS_DESCRIPTOR_SIZE*
                     ROADSIGNS_DESCRIPTOR_ENTRIES;
                 j++) {
                eigen_descriptors[sh][j] =
                    (unsigned char)(eigen[j]/no_of_signs[sh]);
            }
        }
    }
}

void roadsigns_match_descriptor(
    unsigned char * test_descriptor,
    int shape_index,
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int ** sign_matches)
{
    int i,j,n,score;
    unsigned char * eigen, * desc;

    eigen = eigen_descriptors[shape_index];
    desc = descriptors[shape_index];
    for (i=0;i<no_of_signs[shape_index];i++) {
        n = ROADSIGNS_DESCRIPTOR_SIZE*ROADSIGNS_DESCRIPTOR_ENTRIES*i;
        score = 0;
        for (j=0;
             j<ROADSIGNS_DESCRIPTOR_SIZE*ROADSIGNS_DESCRIPTOR_ENTRIES;
             j++,n++) {
            score +=
                255 -
                ABS(((int)test_descriptor[j] - (int)eigen[j]) -
                    ((int)desc[n] - (int)eigen[j]));
        }
        sign_matches[shape_index][i] += score;
    }
}

void roadsigns_identify_sign(
    int image_width,
    int image_height,
    unsigned char * image_road_signs,
    int tx,
    int ty,
    int bx,
    int by,
    int shape_index,
    int * no_of_signs,
    unsigned char ** descriptors,
    unsigned char ** eigen_descriptors,
    int ** sign_matches)
{
    unsigned char test_descriptor[
        ROADSIGNS_DESCRIPTOR_SIZE*ROADSIGNS_DESCRIPTOR_ENTRIES];

    /* create a descriptor */
    roadsigns_descriptor(
        image_width,
        image_height,
        image_road_signs,
        shape_index,
        tx, ty, bx, by,
        test_descriptor);

    /* match */
    roadsigns_match_descriptor(
        test_descriptor,
        shape_index,
        no_of_signs,
        descriptors,
        eigen_descriptors,
        sign_matches);
}

void roadsigns_load_database(
    char * signs_database,
    int * no_of_signs,
    char *** sign_text,
    unsigned char * sign_shapes,
    unsigned char ** sign_descriptor,
    unsigned char ** sign_eigen_descriptor)
{
    DIR *dir;
    char filename[256];
    int x,y,n,max=1;
    int shape_index=0;
    unsigned char * image_data=0;
    struct dirent *ent;

    dir = opendir (signs_database);
    if (dir != 0) {

        while ((ent = readdir (dir)) != 0) {

            if (strlen(ent->d_name)>4) {

                if ((ent->d_name[strlen(ent->d_name)-4]=='.') &&
                    (ent->d_name[strlen(ent->d_name)-3]=='p') &&
                    (ent->d_name[strlen(ent->d_name)-2]=='n') &&
                    (ent->d_name[strlen(ent->d_name)-1]=='g')) {
                    sprintf((char*)filename,"%s/%s",
                        signs_database,ent->d_name);

                    read_png_file(filename);

                    image_data = (unsigned char*)malloc(width*height*3);

                    n=0;
	                for (y=0; y<height; y++) {
  	                    for (x=0; x<width; x++,n+=3) {
		                    image_data[n] =
		                        (unsigned char)(row_pointers[y][x*3]);
		                    image_data[n+1] =
		                        (unsigned char)(row_pointers[y][x*3+1]);
		                    image_data[n+2] =
		                        (unsigned char)(row_pointers[y][x*3+2]);
                        }
                    }

                    /* detect the shape */
                    shape_index = roadsigns_identify_shape(
                        width, height, image_data,
                        0,0,width,height,
                        sign_shapes);

                    if (shape_index>-1) {

                        /* maximum storage */
                        switch(shape_index) {
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

                        /* store name of the sign */
                        sign_text[shape_index]
                            [no_of_signs[shape_index]] =
                            (char*)malloc(strlen(ent->d_name)-3);
                        for (n=0;n<strlen(ent->d_name)-4;n++) {
                            if (ent->d_name[n]!='_') {
                                sign_text[shape_index]
                                    [no_of_signs[shape_index]][n]=
                                        ent->d_name[n];
                            }
                            else {
                                sign_text[shape_index]
                                    [no_of_signs[shape_index]][n]=' ';
                            }
                        }
                        if ((sign_text[shape_index]
                            [no_of_signs[shape_index]][0]>='a') &&
                            (sign_text[shape_index]
                            [no_of_signs[shape_index]][0]<='z')) {
                            sign_text[shape_index]
                            [no_of_signs[shape_index]][0] +=
                            (int)'A'-(int)'a';
                        }
                        sign_text[shape_index]
                            [no_of_signs[shape_index]][n]=0;

                        /* store descriptor */
                        roadsigns_descriptor(
                            width, height, image_data, shape_index,
                            0,0,width-1,height-1,
                            &(sign_descriptor[shape_index]
                                [ROADSIGNS_DESCRIPTOR_SIZE*
                                 ROADSIGNS_DESCRIPTOR_ENTRIES*
                                 no_of_signs[shape_index]]));

                        if (no_of_signs[shape_index] < max)
                            no_of_signs[shape_index]++;
                    }

                    free (image_data);
                }
            }
        }
        closedir (dir);

        /* update eigen descriptors */
        roadsigns_update_eigen_descriptors(
            no_of_signs,sign_descriptor,sign_eigen_descriptor);
    }
}

void roadsigns_show(
    int image_width,
    unsigned char * image_data,
    int shape_index,
    int tx,
    int ty,
    int bx,
    int by)
{
    int x,y,n;

    for (y=ty;y<=by;y++) {
        n = (y*image_width+tx)*3;
        image_data[n]=255;
        image_data[n+1]=0;
        image_data[n+2]=0;
        n = (y*image_width+bx)*3;
        image_data[n]=255;
        image_data[n+1]=0;
        image_data[n+2]=0;
    }
    for (x=tx;x<=bx;x++) {
        n = (ty*image_width+x)*3;
        image_data[n]=255;
        image_data[n+1]=0;
        image_data[n+2]=0;
        n = (by*image_width+x)*3;
        image_data[n]=255;
        image_data[n+1]=0;
        image_data[n+2]=0;
    }
}

