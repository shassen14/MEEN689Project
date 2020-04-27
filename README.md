# MEEN 689 Project README

## Team Members:
- Akshay Sarvesh
- DJ Franklin
- Jadha Gunawan
- Samir Hassen

Please see the .txt file in the release for the corresponding UIN’s, which is emailed to sri@fastmail.fm

## How to run the code
To run the code for our project, please get the most recent release of this project and run the  “EKF_code.m” file in Matlab.  This function will generate all the necessary plots. The code uses “drive1.mat” for our raw sensor data.

## Description of Figures
Figure 1 shows  a plot of the position of the phone as the car was driven around.  It shows the measured GPS coordinates as green circles and the estimated position of the phone as a blue line.  The dimensions of the graph are in meters, and the data is in the global ENU reference frame.  The origin is defined to be wherever the phone started recording data.  The car drove clockwise in a loop, starting from the origin and returning to its original position.

![figure 1](/graphs/position.jpg)

Figure 2 shows the yaw angle of the phone over time in degrees, as estimated from the kalman filter based on the magnetometer reading.

![figure 2](/graphs/yaw.jpg)

Figure 3 shows the local velocity of the phone over time, as estimated by the filter.  The vertical axis is in meters per second, and the horizontal axis shows time in seconds.  The blue line shows the velocity in the local x-direction, and the red line shows the velocity in the local y-direction.  Because the phone was placed facing forward in the vehicle, the x-velocity averages around zero, while the y-velocity should show how fast the car was moving forward.

![figure 3](/graphs/vel_local.jpg)

Figure 4 shows the global velocity of the phone, as estimated by the filter.

![figure 4](/graphs/vel_global.jpg)

Figure 5 shows the local acceleration from the phone, as measured by the accelerometer.

![figure 5](/graphs/acc_local.jpg)

Figure 6 shows the global acceleration of the phone, as estimated by the kalman filter.

![figure 6](/graphs/acc_global.jpg)

Lastly, figure 7 shows the magnitude of the velocity of the phone.  The blue line shows the speed measured by the GPS, and the orange line shows the speed estimate by the kalman filter.

![figure 7](/graphs/speed.jpg)
