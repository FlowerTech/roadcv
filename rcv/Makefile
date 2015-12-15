
all:
	gcc -Wall -ansi -pedantic -O3 -o rcv -I'/usr/include/libpng' -lpng rcv.c roadcv.c vehiclegeometry.c road.c roadmarkings.c vehicledetect.c roadsigns.c floodfill.c som.c numberimages.c warnings.c trafficlights.c -fopenmp
 
debug:
	gcc -Wall -ansi -pedantic -g -o rcv -I'/usr/include/libpng' -lpng rcv.c roadcv.c vehiclegeometry.c road.c roadmarkings.c vehicledetect.c roadsigns.c floodfill.c som.c numberimages.c warnings.c trafficlights.c -fopenmp

clean:
	rm -f rcv
	rm -f *.png
	rm -f *.deb
	rm -rf deb/usr

