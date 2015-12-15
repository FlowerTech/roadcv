/*
 self organising map
 This is used for classifying binary patterns - typically binary images
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

#include "som.h"

/* An initial few bytes contains information about the SOM */
#define SOM_OFFSET 4

#define SOM_MATCH_BYTE(v) \
    (((v)&1?1:0)+((v)&2?1:0)+((v)&4?1:0)+((v)&8?1:0)+\
    ((v)&16?1:0)+((v)&32?1:0)+((v)&64?1:0)+((v)&128?1:0))

void som_draw(
    unsigned char * img,
    int img_width,
    int img_height,
    unsigned char * map,
    unsigned char * input)
{
    int ip,n,i,x,y,xx,yy,mag,inputs;
    unsigned char dim;
    unsigned char * inputinv;
    dim = map[0]; /* size of the map */
    inputs = map[2];

    inputinv = (unsigned char*)malloc(inputs*sizeof(unsigned char));

    /* invert inputs for anticorrelation match */
    for (ip=0;ip<inputs;ip++) {
        n = input[ip];
        inputinv[ip] = 0;
        for (i = 0; i < 8; i++) {
            inputinv[ip] <<= 1;
            inputinv[ip] |= n & 1;
            n >>= 1;
        }
    }

    n=0;
    for (y=0;y<img_height;y++) {
        yy = y * (dim-1) / img_height;
        for (x=0;x<img_width;x++,n+=3) {
            xx = x * (dim-1) / img_width;
            i = yy*dim+xx;
            mag=inputs*8;
            for (ip=0;ip<inputs;ip++) {
                mag += SOM_MATCH_BYTE(
                    map[i*inputs+SOM_OFFSET+ip] & input[ip]);
                mag -= SOM_MATCH_BYTE(
                    map[i*inputs+SOM_OFFSET+ip] & inputinv[ip]);
            }
            img[n] = mag*8;
            img[n+1] = mag*8;
            img[n+2] = mag*8;
        }
    }
    free(inputinv);
}

unsigned char som_cycle(
    unsigned char * map,
    unsigned char * input,
    int reinforcement) /* in the range -16384 -> 16384 */
{
    unsigned char x,y,b;
    int p;
    int ip,i,n,inputs,radius,retval = 0;
    unsigned char dim,mx=0,my=0;
    unsigned char * input2;
    unsigned char * inputinv;
    int mag,max=-9999;
    dim = map[0]; /* size of the map */
    radius = map[1]; /* update radius */
    inputs = map[2]; /* input bytes */

    inputinv = (unsigned char*)malloc(inputs*sizeof(unsigned char));

    /* invert inputs for anticorrelation match */
    for (ip=0;ip<inputs;ip++) {
        n = input[ip];
        inputinv[ip] = 0;
        for (i = 0; i < 8; i++) {
            inputinv[ip] <<= 1;
            inputinv[ip] |= n & 1;
            n >>= 1;
        }
    }

    for (i=0; i < dim*dim; i++) {
        mag=0;
        n = i*inputs+SOM_OFFSET;
        for (ip=0;ip<inputs;ip++,n++) {
            mag += SOM_MATCH_BYTE(map[n] & input[ip]);
            mag -= SOM_MATCH_BYTE(map[n] & inputinv[ip]);
        }
        if (mag > max) {
            max = mag;
            retval = i;
        }
    }

    /* create a value for peak response which
       can be fed into other soms */
    my = retval / dim;
    mx = retval % dim;
    retval=0;
    n=1;
    for (i=0;i<MULDIVINT32(mx,4,dim);i++,n*=2) retval |= n;
    n=16;
    for (i=0;i<MULDIVINT32(my,4,dim);i++,n*=2) retval |= n;

    if (reinforcement!=0) {

        input2 = input;
        if (reinforcement < 0) {
            /* make the response less similar to the input */
            input2 = inputinv;
            reinforcement = -reinforcement;
        }

        /* update the hood */
        for (y=my-radius;y<=my+radius;y++) {
            for (x=mx-radius;x<=mx+radius;x++) {
                n=((y%dim)*dim+(x%dim))*inputs+SOM_OFFSET;

                /* lower probability in the surround */
                p = reinforcement;
                /* higher probability at the centre */
                if (n==retval) p = reinforcement<<1;

                for (ip=0;ip<inputs;ip++) {
                    b=0;
                    i=128;
                    while (i>0) {
                        if (rand()%16384 < p) {
                            if (input2[ip]&i) b|=i;
                        }
                        else {
                            if (map[n+ip]&i) b|=i;
                        }
                        i>>=1;
                    }
                    map[n+ip] = b;
                }

            }
        }
    }

    free(inputinv);

    return (unsigned char)retval;
}

void som_init(
    unsigned char * map,
    unsigned char dim,
    unsigned char radius,
    unsigned char input_bits,
    int seed)
{
    int i;
    map[0] = dim;           /* dimension of the map */
    map[1] = radius;        /* activation radius */
    map[2] = input_bits>>3; /* number of input bytes */

    /* initial random distribution */
    srand(seed);
    for (i=SOM_OFFSET;i<dim*dim*map[2]+SOM_OFFSET;i++) {
        map[i] = (unsigned char)(rand() % 256);
    }
}

