# Converting GPS Measurements to Meters

Right now, my phone is telling me that I am at about 30.623006 degrees latitude and -96.344881 degrees longitude.  These equations from [Wikipedia](https://en.wikipedia.org/wiki/Geographic_coordinate_system) give how many meters long a degree latitude and a degree longitude are, at a specific latitude phi (where phi is in degrees), for small changes in latitude and longitude.

1 degree of latitude = 111132.92 - 559.82cos(2phi) + 1.175cos(4phi) - 0.0023cos(6phi) meters

1 degree of longitude = 111412.84cos(phi) - 93.5cos(3phi) + 0.118cos(5phi) meters

This means that 1 degree of latitude is about 110862.9887 meters, and 1 degree of longitude is about 95877.9400 meters.  I kept the extra digits so we don't round our intermediate calculations too much.

We can convert our latitude and longitude from decimal degrees to degrees, minutes, and seconds using the "degrees2dms()" function in matlab.   Using the function shows that our latitude is 30 degrees, 37 minutes, and 22.595 seconds, while our longitude is -96 degrees, 20 minutes, and 41.298 seconds.  
