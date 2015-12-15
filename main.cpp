/*
    roadcv
    A command line utility for vehicular vision
    Copyright (C) 2010 Bob Mottram
    fuzzgun@gmail.com

    Requires packages:
		libgstreamer-plugins-base0.10-dev
		libgst-dev

    sudo apt-get install
        libcv4 libhighgui4 libcvaux4 libcv-dev
        libcvaux-dev libhighgui-dev
        libgstreamer-plugins-base0.10-dev libgst-dev

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.
    If not, see <http://image_widthw.gnu.org/licenses/>.
*/

//#define ENABLE_GSTREAMER

/* Note that for GPS operation you will need to install gpsd and libgps-dev
   sudo apt-get install gpsd libgps-dev */
#define ENABLE_GPS

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#ifdef ENABLE_GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappbuffer.h>
#include <sstream>
#endif
#include <omp.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>

#include "anyoption.h"
#include "libcam.h"

#include "rcv/roadcv.h"
#ifdef ENABLE_GPS
#include "rcv/gpsdata.h"
#endif

#define VIEW_RAW            0
#define VIEW_GROUND_PLANE   1
#define VIEW_SEGMENTED      2
#define VIEW_VEHICLES       3

#define VERSION 0.1

using namespace std;

int main(int argc, char* argv[]) {
  int fps,skip_frames = 1;
  int fov=75,tilt=4;
  int image_width=320,image_height=240;
  int viewing_mode=VIEW_RAW;
  int image_index = 0;
  unsigned char * buffer=NULL;
  char * signs_database=NULL;
  int wait, x,y, x2,y2, n0,n1;
  unsigned char detect_signs=0,detect_vehicles=0,enable_audio=0;
  unsigned char show_lanes=0,headless=0,detect_traffic_lights=0;
  int save_period_sec,r,g,b,minimum_lane_crossing_warning_speed_mph;
  int lane_crossing_tollerance_percent;
  int projected_width, projected_height;
  bool show_vehicles;
  bool stream,flip_image,save_images;
  int camera_height_mm = 1000;
  std::string save_filename;
  std::string dev0;
  std::string left_image_title;
  char * sounds_path = NULL;
  AnyOption *opt;
  struct rcv_data rcv;

#ifdef ENABLE_GPS
  double longitude=0,latitude=0,speed=0;
#endif

  IplImage *l;
  unsigned char *l_;
#ifdef ENABLE_GSTREAMER
  GstElement* l_source;
  GstBuffer* l_app_buffer;
  GstFlowReturn ret;
  std::string caps;
  std::stringstream lp_str;

  /* Port to start streaming from - second video will be on this + 1 */
  int start_port = 5000;
#endif

#ifdef ENABLE_GPS
  if (gpsdata_start()==0) {
      printf("Could not connect to GPS device.  Try running startgps.sh\n");
      return 0;
  }
#endif

  opt = new AnyOption();
  assert(opt != NULL);

  /* help */
  opt->addUsage( "Example: " );
  opt->addUsage( "  roadcv -v /dev/video1 -w 320 -h 240 --lanes --vehicles" );
  opt->addUsage( " " );
  opt->addUsage( "Usage: " );
  opt->addUsage( "" );
  opt->addUsage( " -v  --video      Video device, eg. /dev/video0");
  opt->addUsage( " -w  --width      Image width in pixels");
  opt->addUsage( " -h  --height     Image height in pixels");
  opt->addUsage( " -e  --elevation  Height of camera above the ground in mm");
  opt->addUsage( "     --fov        Field of view in degrees");
  opt->addUsage( "     --tilt       Camera tilt in degrees");
  opt->addUsage( "     --fps        Frames per second");
  opt->addUsage( " -s  --skip       Skip this number of frames");
  opt->addUsage( " -V  --version    Show version number");
  opt->addUsage( "     --projectedwidth   Width of the projected image");
  opt->addUsage( "     --projectedheight  Height of the projected image");
  opt->addUsage( "     --save       Save raw images");
  opt->addUsage( "     --saveperiod Save images repeatedly every x seconds");
  opt->addUsage( "     --flip       Flip the image");
  opt->addUsage( "     --ground     View ground plane (overhead view)");
  opt->addUsage( "     --segmented  View segmented");
  opt->addUsage( "     --lanes      Show lanes");
  opt->addUsage( "     --lanespeed  Minimum speed (mph) above which lane");
  opt->addUsage( "                  crossing warnings become active");
  opt->addUsage( "     --cross      Tollerance percent for lane crossing");
  opt->addUsage( "     --vehicles   Detect vehicles");
  opt->addUsage( "     --signs      Detect signs");
  opt->addUsage( "     --lights     Detect traffic lights");
  opt->addUsage( "     --sounds     Path to sound files");
  opt->addUsage( "     --signsdata  Signs database");
  opt->addUsage( "     --stream     Stream output using gstreamer");
  opt->addUsage( "     --headless   Disable video output");
  opt->addUsage( "     --help                     Show help");
  opt->addUsage( "" );

  opt->setOption(  "saveperiod" );
  opt->setOption(  "save" );
  opt->setOption(  "fps" );
  opt->setOption(  "elevation", 'e' );
  opt->setOption(  "video", 'v' );
  opt->setOption(  "width", 'w' );
  opt->setOption(  "height", 'h' );
  opt->setOption(  "skip", 's' );
  opt->setOption(  "fov" );
  opt->setOption(  "tilt" );
  opt->setOption(  "signsdata" );
  opt->setOption(  "sounds" );
  opt->setOption(  "lanespeed" );
  opt->setOption(  "cross" );
  opt->setOption(  "projectedwidth" );
  opt->setOption(  "projectedheight" );
  opt->setFlag(  "help" );
  opt->setFlag(  "flipleft" );
  opt->setFlag(  "flipright" );
  opt->setFlag(  "version", 'V' );
  opt->setFlag(  "stream"  );
  opt->setFlag(  "headless"  );
  opt->setFlag(  "ground"  );
  opt->setFlag(  "segmented"  );
  opt->setFlag(  "lanes"  );
  opt->setFlag(  "signs"  );
  opt->setFlag(  "lights"  );
  opt->setFlag(  "vehicles"  );

  opt->processCommandArgs(argc, argv);

  if(!opt->hasOptions()) {
      /* print usage if no options */
      opt->printUsage();
      delete opt;
      return(0);
  }

  if((opt->getFlag("version")) ||
     (opt->getFlag('V'))) {
      printf("Version %f\n", VERSION);
      delete opt;
      return(0);
  }

  stream = false;
  if(opt->getFlag("stream")) {
	  stream = true;
  }

  headless = 0;
  if(opt->getFlag("headless")) {
      headless = 1;
  }

  if(opt->getFlag("lanes")) {
      show_lanes = 1;
  }

  if(opt->getFlag("signs")) {
      detect_signs = 1;
  }

  if(opt->getFlag("lights")) {
      detect_traffic_lights = 1;
  }

  if(opt->getFlag("audible")) {
      enable_audio = 1;
  }

  if(opt->getFlag("vehicles")) {
      detect_vehicles = 1;
  	  viewing_mode = VIEW_VEHICLES;
  }

  if(opt->getFlag("ground")) {
	  viewing_mode = VIEW_GROUND_PLANE;
  }

  if(opt->getFlag("segmented")) {
	  viewing_mode = VIEW_SEGMENTED;
  }

  flip_image = false;
  if(opt->getFlag("flip")) {
	  flip_image = true;
  }

  if(opt->getValue("sounds") != NULL) {
      enable_audio=1;
  	  sounds_path = opt->getValue("sounds");
  }

  save_images = false;
  save_filename = "";
  if(opt->getValue("save") != NULL) {
  	  save_filename = opt->getValue("save");
  	  if (save_filename == "") save_filename = "image_";
  	  save_images = true;
  }

  minimum_lane_crossing_warning_speed_mph=50;
  if(opt->getValue("lanespeed") != NULL) {
      minimum_lane_crossing_warning_speed_mph =
          atoi(opt->getValue("lanespeed"));
  }

  lane_crossing_tollerance_percent=20;
  if(opt->getValue("cross") != NULL) {
      lane_crossing_tollerance_percent =
          atoi(opt->getValue("cross"));
  }

  if(opt->getValue("signsdata") != NULL) {
  	  signs_database = opt->getValue("signsdata");
  }

  if(opt->getFlag("help") ) {
      opt->printUsage();
      delete opt;
      return(0);
  }

  projected_width = 160;
  if(opt->getValue("projectedwidth") != NULL) {
	  projected_width = atoi(opt->getValue("projectedwidth"));
  }

  projected_height = 240;
  if(opt->getValue("projectedheight") != NULL) {
	  projected_height = atoi(opt->getValue("projectedheight"));
  }

  save_period_sec = 0;
  if(opt->getValue("saveperiod") != NULL) {
	  save_period_sec = atoi(opt->getValue("saveperiod"));
	  if (save_period_sec < 1) save_period_sec=1;
  }

  if(opt->getValue("fov") != NULL) {
  	  fov = atoi(opt->getValue("fov"));
  }

  dev0 = "/dev/video1";
  if((opt->getValue('v') != NULL) ||
     (opt->getValue("video") != NULL)) {
  	  dev0 = opt->getValue("video");
  }

  if((opt->getValue('w') != NULL) ||
     (opt->getValue("width") != NULL)) {
  	  image_width = atoi(opt->getValue("width"));
  }

  if((opt->getValue('h') != NULL) ||
     (opt->getValue("height") != NULL)) {
  	  image_height = atoi(opt->getValue("height"));
  }

  fps = 30;
  if(opt->getValue("fps") != NULL) {
	  fps = atoi(opt->getValue("fps"));
  }

  if((opt->getValue('e') != NULL) ||
     (opt->getValue("elevation") != NULL)) {
	  camera_height_mm = atoi(opt->getValue("elevation"));
  }

  if(opt->getValue("tilt") != NULL) {
	  tilt = atoi(opt->getValue("tilt"));
  }

  if((opt->getValue('s') != NULL) ||
     (opt->getValue("skip") != NULL)) {
	  skip_frames = atoi(opt->getValue("skip"));
  }

  delete opt;

  Camera c(dev0.c_str(), image_width, image_height, fps);

  left_image_title = "Image";

  if ((!save_images) &&
	  (!headless!=0)) {
      cvNamedWindow(left_image_title.c_str(), CV_WINDOW_AUTOSIZE);
  }

  l=cvCreateImage(cvSize(image_width, image_height), 8, 3);
  l_=(unsigned char *)l->imageData;

#ifdef GSTREAMER_ENABLED
  /*
   * Send the video over a network for use in embedded applications
   * using the gstreamer library.
   */
  l_source = NULL;
  l_app_buffer = NULL;

  lp_str << start_port;

  if( stream ) {
	/* Initialise gstreamer and glib */
    gst_init( NULL, NULL );
	GError* l_error = 0;
	GstElement* l_pipeline = 0;

    caps = "image/jpeg";

	/* Can replace this pipeline with anything you
	   like (udpsink, videowriters etc) */
	std::string l_pipetext =
	    "appsrc name=appsource caps="+ caps +
	    " ! jpegdec ! ffmpegcolorspace ! queue ! jpegenc"+
	    " ! multipartmux ! tcpserversink port=" + lp_str.str();

	/* Create the left image pipeline */
	l_pipeline = gst_parse_launch( l_pipetext.c_str(), &l_error );

	/* Seperate errors in case of port clash */
	if( l_error == NULL ) {
	    l_source =
	        gst_bin_get_by_name( GST_BIN( l_pipeline ), "appsource" );
	    gst_app_src_set_caps(
	        (GstAppSrc*) l_source,
	        gst_caps_from_string( caps.c_str() ) );
	    gst_element_set_state( l_pipeline, GST_STATE_PLAYING );
	    cout << "Streaming started on port " << start_port << endl;
	    cout << "Watch stream with the command:" << endl;
	    cout << "gst-launch tcpclientsrc host=[ip] port=" <<
	        start_port <<
	        " ! multipartdemux ! jpegdec ! autovideosink" << endl;
	} else {
	    cout << "A gstreamer error occurred: " << l_error->message << endl;
	}
  }
#endif

  /* initialise roadcv */
  roadcv_open(
    image_width, image_height,
    projected_width, projected_height,
    l_,
    1, fov, tilt, camera_height_mm,
    (int)detect_vehicles,
    (int)detect_signs,
    (int)detect_traffic_lights,
    (int)enable_audio,
    sounds_path,
    signs_database,
    &rcv,
    1,
    (int)headless,
    (int)show_lanes,
    minimum_lane_crossing_warning_speed_mph,
    lane_crossing_tollerance_percent);

  assert(rcv.vehicle_width_mm==2000);

  while(1){

    while(c.Get()==0) usleep(100);

    c.toIplImage(l);

    if (flip_image) {
    	if (buffer == NULL) {
    		buffer = new unsigned char[image_width * image_height * 3];
    	}
        n0=0;
        for (y=0;y<image_height;y++) {
            for (x=0;x<image_width;x++,n0+=3) {
                n1 = ((image_height-1-y)*image_width+x)*3;
                buffer[n1]=l_[n0];
                buffer[n1+1]=l_[n0+1];
                buffer[n1+2]=l_[n0+2];
            }
        }
    }

#ifdef ENABLE_GPS
    gpsdata_update(&longitude,&latitude,&speed);
    rcv.longitude = (int)(longitude*GEOCOORD_MULT);
    rcv.latitude = (int)(latitude*GEOCOORD_MULT);
    rcv.speed_mph = gpsdata_miles_per_hour();
    rcv.stopping_distance_mm =
        vehicledetect_stopping_distance(rcv.speed_mph);
#endif

    roadcv_update(&rcv);

    if (headless==0) {
        switch(viewing_mode) {
            case VIEW_RAW: {
                if ((detect_signs != 0) &&
                    (rcv.sign_region[0] > 0)) {
                    roadsigns_show(
                        image_width,
                        rcv.image_data,
                        rcv.shape_index,
                        rcv.sign_region[0],
                        rcv.sign_region[1],
                        rcv.sign_region[2],
                        rcv.sign_region[3]);
                }
                break;
            }
            case VIEW_GROUND_PLANE: {
                n0 = 0;
                for (y=0;y<image_height;y++) {
                    y2 = y*rcv.projected_height/image_height;
                    for (x=0;x<image_width;x++,n0+=3) {
                        x2 = x*rcv.projected_width/image_width;
                        n1 = (y2*rcv.projected_width+x2)*3;
                        l_[n0] = rcv.image_ground_plane[n1];
                        l_[n0+1] = rcv.image_ground_plane[n1+1];
                        l_[n0+2] = rcv.image_ground_plane[n1+2];
                    }
                }
                break;
            }
            case VIEW_SEGMENTED: {
                road_reproject(
                    rcv.projected_width, rcv.projected_height,
                    rcv.image_segmented,
                    rcv.image_width, rcv.image_height,
                    rcv.reprojected_lookup, rcv.image_data);

                memcpy(
                    (void*)l_,
                    (void*)rcv.image_data,
                    image_width*image_height*3);
                break;
            }
            case VIEW_VEHICLES: {
                if ((rcv.detect_vehicles != 0) &&
                    (rcv.no_of_possible_vehicles>0)) {

                    /* bounding box turns red if the vehicle
                       is inside the stopping distance */
                    if (rcv.warnings & WARNING_STOPPING_DISTANCE) {
                        r=255;
                        g=b=0;
                    }
                    else {
                        b=0;
                        r=g=255;
                    }

                    vehicledetect_show_vehicles(
                        rcv.image_width, rcv.image_height,
                        rcv.image_data,
                        rcv.no_of_possible_vehicles[0],
                        rcv.possible_vehicles[0],
                        b,g,r);

                    memcpy(
                        (void*)l_,
                        (void*)rcv.image_data,
                        image_width*image_height*3);
                }
                break;
            }
        }
        /* show detected traffic lights */
        if (rcv.detect_traffic_lights!=0) {
            trafficlights_show(
                rcv.image_width, rcv.image_height,
                rcv.image_data,
                rcv.no_of_traffic_lights[0],
                rcv.traffic_lights[0], 1);
        }
    }

	if (skip_frames == 0) {

		if (save_period_sec > 0) {
			char filename[256];
			sprintf((char*)filename,"image_%d.png", image_index);
			cvSaveImage(filename, l);
			image_index++;
			sleep(save_period_sec);
		}

		/* save image to file, then quit */
		if (save_images) {
			std::string filename = save_filename + ".png";
			cvSaveImage(filename.c_str(), l);
			break;
		}

	}

#ifdef ENABLE_GSTREAMER
    /*
     * The streaming bit - seems a bit hacky, someone else can try
     * and convert an IPLImage directly to something GStreamer can handle.
     * My bitbanging abilities just aren't up to the task.
     */
    if (stream) {
	    CvMat* l_buf;
	    l_buf = cvEncodeImage(".jpg", l);

	    l_app_buffer =
	        gst_app_buffer_new(
	            l_buf->data.ptr,
	            l_buf->step,
	            NULL,
	            l_buf->data.ptr );
	    g_signal_emit_by_name(
	        l_source,
	        "push-buffer",
	        l_app_buffer,
	        &ret );
    }
#endif

	/* display the left and right images */
	if ((!save_images) && (headless==0)) {
	    cvShowImage(left_image_title.c_str(), l);
	}

    skip_frames--;
    if (skip_frames < 0) skip_frames = 0;

    wait = cvWaitKey(10) & 255;
    if( wait == 27 ) break;
  }

  /* destroy the left and right images */
  if (!save_images) {
	  cvDestroyWindow(left_image_title.c_str());
  }
  cvReleaseImage(&l);
  if (buffer != NULL) delete[] buffer;

  roadcv_close(&rcv);

#ifdef ENABLE_GPS
  gpsdata_stop();
#endif

  return 0;
}

