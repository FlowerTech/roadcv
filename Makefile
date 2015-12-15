
all:
	g++ -O3 -o roadcv main.cpp anyoption.cpp libcam.cpp -I/usr/include/opencv -I/usr/include/libpng -L/usr/lib -lgps -lcv -lcxcore -lcvaux -lhighgui -lpng rcv/vehiclegeometry.c rcv/road.c rcv/roadmarkings.c rcv/vehicledetect.c rcv/roadcv.c rcv/roadsigns.c rcv/floodfill.c rcv/som.c rcv/numberimages.c rcv/gpsdata.c rcv/warnings.c rcv/trafficlights.c -fopenmp

gst:
	g++ -O3 -o roadcv main.cpp anyoption.cpp libcam.cpp -I/usr/include/opencv -I/usr/include/libpng `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lgps -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -lpng rcv/vehiclegeometry.c rcv/road.c rcv/roadmarkings.c rcv/vehicledetect.c rcv/roadcv.c rcv/roadsigns.c rcv/floodfill.c rcv/som.c rcv/numberimages.c rcv/gpsdata.c rcv/warnings.c rcv/trafficlights.c -fopenmp
 

debug:
	g++ -g -o roadcv main.cpp anyoption.cpp libcam.cpp -I/usr/include/opencv -I/usr/include/libpng -L/usr/lib -lgps -lcv -lcxcore -lcvaux -lhighgui -lpng rcv/vehiclegeometry.c rcv/road.c rcv/roadmarkings.c rcv/vehicledetect.c rcv/roadcv.c rcv/roadsigns.c rcv/floodfill.c rcv/som.c rcv/numberimages.c rcv/gpsdata.c rcv/warnings.c rcv/trafficlights.c -fopenmp

gstdebug:
	g++ -g -o roadcv main.cpp anyoption.cpp libcam.cpp -I/usr/include/opencv -I/usr/include/libpng `pkg-config --cflags --libs gstreamer-0.10` -L/usr/lib -lgps -lcv -lcxcore -lcvaux -lhighgui `pkg-config --cflags --libs glib-2.0` `pkg-config --cflags --libs gstreamer-plugins-base-0.10` -lgstapp-0.10 -lpng rcv/vehiclegeometry.c rcv/road.c rcv/roadmarkings.c rcv/vehicledetect.c rcv/roadcv.c rcv/roadsigns.c rcv/floodfill.c rcv/som.c rcv/numberimages.c rcv/gpsdata.c rcv/warnings.c rcv/trafficlights.c -fopenmp

clean:
	rm -f roadcv
	rm -rf deb/usr
	rm -f *.deb

