/*
 libpng interface
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

#ifndef PNG2_H
#define PNG2_H

#define PNG_DEBUG 3
#include <png.h>

static int width, height;
static png_byte color_type;
static png_byte bit_depth;
static png_structp png_ptr=NULL;
static png_infop info_ptr=NULL;
static int number_of_passes;
static png_bytep * row_pointers;
static unsigned char png_loaded = 0;

static void read_png_file(
    char* file_name)
{
	char header[8];	/* 8 is the maximum size that can be checked */
    FILE *fp;
    int ret,y;

	/*if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);*/

	/* open file and test for it being a png */
	fp = fopen(file_name, "rb");
	if (!fp) {
		printf("[read_png_file] File ");
		printf("%s could not be opened for reading", file_name);
    }
	ret = fread(header, 1, 8, fp);
	if (png_sig_cmp((png_bytep)header, (png_size_t)0, (png_size_t)8)) {
		printf("[read_png_file] File ");
		printf("%s is not recognized as a PNG file", file_name);
    }

	/* initialize stuff */
	png_ptr =
	    png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	
	if (!png_ptr)
		printf("[read_png_file] png_create_read_struct failed");

	info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		printf("[read_png_file] png_create_info_struct failed");

	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[read_png_file] Error during init_io");

	png_init_io(png_ptr, fp);
	png_set_sig_bytes(png_ptr, 8);

	png_read_info(png_ptr, info_ptr);

	width = info_ptr->width;
	height = info_ptr->height;
	color_type = info_ptr->color_type;
	bit_depth = info_ptr->bit_depth;

   if (bit_depth > 8) {
      png_set_strip_16(png_ptr);
   }
   if (color_type == PNG_COLOR_TYPE_GRAY ||
       color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
      png_set_gray_to_rgb(png_ptr);
   }
   if (color_type == PNG_COLOR_TYPE_PALETTE) {
      png_set_palette_to_rgb(png_ptr);
   }
	color_type = info_ptr->color_type;
	bit_depth = info_ptr->bit_depth;

	number_of_passes = png_set_interlace_handling(png_ptr);
	png_read_update_info(png_ptr, info_ptr);

	/* read file */
	if (setjmp(png_jmpbuf(png_ptr)))
		printf("[read_png_file] Error during read_image");

	row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
	for (y=0; y<height; y++)
		row_pointers[y] = (png_byte*) malloc(info_ptr->rowbytes);

	png_read_image(png_ptr, row_pointers);

    fclose(fp);
    png_loaded = 1;
}

static int write_png_file(
    char* filename,
    int width,
    int height,
    unsigned char *buffer)
{
	int x, y, i,code = 0;
	FILE *fp;
	png_structp png_ptr;
	png_infop info_ptr=0;
	png_bytep row=0;
	
	/* Open file for writing (binary mode) */
	fp = fopen(filename, "wb");
	if (fp == NULL) {
		fprintf(stderr, "Could not open file %s for writing\n", filename);
		code = 1;
		goto finalise;
	}

	/* Initialize write structure */
	png_ptr =
	    png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (png_ptr == NULL) {
		fprintf(stderr, "Could not allocate write struct\n");
		code = 1;
		goto finalise;
	}

	/* Initialize info structure */
	info_ptr = png_create_info_struct(png_ptr);
	if (info_ptr == NULL) {
		fprintf(stderr, "Could not allocate info struct\n");
		code = 1;
		goto finalise;
	}

	/* Setup Exception handling */
	if (setjmp(png_jmpbuf(png_ptr))) {
		fprintf(stderr, "Error during png creation\n");
		code = 1;
		goto finalise;
	}

	png_init_io(png_ptr, fp);

	/* Write header (8 bit colour depth) */
	png_set_IHDR(png_ptr, info_ptr, width, height,
			8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

	png_write_info(png_ptr, info_ptr);

	/* Allocate memory for one row (3 bytes per pixel - RGB) */
	row = (png_bytep) malloc(3 * width * sizeof(png_byte));

	/* Write image data */
	i=0;
	for (y=0 ; y<height ; y++) {
		for (x=0 ; x<width ; x++, i+=3) {
		    png_byte* ptr = &(row[x*3]);
            ptr[0] = buffer[i];
            ptr[1] = buffer[i+1];
            ptr[2] = buffer[i+2];
		}
		png_write_row(png_ptr, row);
	}

	png_write_end(png_ptr, NULL);

	finalise:
	if (fp != NULL) fclose(fp);
	if (info_ptr != NULL) {
	    png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
	}
	if (png_ptr != NULL) {
	    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
	}
	if (row != NULL) free(row);

	return code;
}

#endif
