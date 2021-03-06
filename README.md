Roadcv
======

1.0  Introduction
-----------------

In 2010 there didn't appear to be any open source automotive vision systems
available.  Searching the internet came up with to main categories:

i)  People experimenting with OpenCV and other unspecified algorithms.
ii) Some commercial systems.

OpenCV is ok, but not well suited to an automotive system.  My main concern
with OpenCV is that the heavy reliance upon floating point arithmetic probably
means that any system implemented using it either will not run, or will run
badly, on low end embedded hardware such as ARM based single board computers
or DSPs.  To be practical for an embedded system, probably all of the maths
being executed during live operation should be integer only and optimised
as far as possible.

Little tangible information was available on the commercial systems (even
including their cost), and the demonstration videos shown all appear to be
obtained under what I would describe as ideal viewing conditions, where the
road markings are extremely clear and unambiguous and there are no reflections,
puddles, shadows or other distractions.  This lead me to suspect that
the commercial systems available in 2010 were not as robust as the
brief demo videos might have implied.

There are also other concerns with closed source automotive vision systems.
Since these systems are intended primarily to improve diving safety the
lack of an independently auditable and modifiable code base is worrying.
Where systems are safety critical there should be as much transparency of the
workings of the system as possible so that bugs can be located and fixed quickly
to avoid accidents, and any improvements can be added.

So this appears to be fertile territory for an open source methodology, where
many users can test and debug the system under a wide variety of driving
conditions.

2.0  How can I help to improve the system ?
-------------------------------------------

2.1  Gather test images
-----------------------

One of the best ways in which users can help to improve the system is to
gather image data sets.  Currenly all testing has been carried out in a
vehicle where the camera is located about one metre above the ground.  It
would be good to also obtain data sets for different sized vehicles such as
vans and trucks where the camera is higher, and under a variety of times
of day, types of road, weather conditions and camera models.

To gather data position the camera in the centre of the dashboard pointing
directly forwards and possibly angled slightly downwards, such that the horizon
appears somewhere in upper half of the image and only the minimum of dashboard
can be seen at the bottom of the image.  Then run:

    roadcv -v /dev/video0 --saveperiod 1

Optionally you can add the headless option if you don't want to see the image.

    roadcv -v /dev/video0 --saveperiod 1 --headless

This will save an image approximately every one second.  Drive some short
distance then stop, exit the program with CTRL-C or Escape, then compress the
saved images into an archive.

2.2 Internationalisation
------------------------

The system is currently being tested with UK road signs and traffic lights,
but signs will vary from one country to another and it would be good to be
able to obtain images of road signs from as many places as possible.

2.3 Testing on embedded hardware
--------------------------------

If you have a small embedded computer with a reasonably fast CPU capable of
running some version of Linux (preferebly but not necessarily Debian based)
and with the facility for audio output so that various warning sounds can be
played then you can help by testing the software on your device.  Low
cost netbooks or tablets might also be a fairly inexpensive way of running
the software.


3.0  GPS Devices
----------------

In order to estimate the current speed of the vehicle an attached GPS
receiver is used in combination with the gpsd daemon.

Firstly ensure that you have gpsd installed.  This is normally installed
as part of the roadcv deb package, but if you are intending to compile
from source you will need to install it manually.

    sudo apt-get install gpsd libgps-dev

To run the gps daemon either run the script

    sh startgps.sh

or

    stty 38400 < /dev/ttyUSB0
    gpsd -N -D 2 /dev/ttyUSB0
