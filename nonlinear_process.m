function x_hat_minus = nonlinear_process(dt, xk, fastMeasurements)
%nonlinear_process is the nonlinear x=A(x) process model

%here we extract our yaw angle and our measurements.
theta = xk(7);
acc_x = fastMeasurements(1);
acc_y = fastMeasurements(2);
gyro = fastMeasurements(3);
mag_x = fastMeasurements(4);
mag_y = fastMeasurements(5);
orient = fastMeasurements(6); %we don't use this, since it's not a real
%measurement, but if we did want it, it would be here for us.

%now we make a rotation matrix to rotate the measured acceleration into the
%inertial frame.
R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

%now we define the new x & y position, velocity, acceleration, and yaw.
new_xy = [xk(1);xk(2)] + [xk(3);xk(4)]*dt + R*[acc_x;acc_y]/2*dt^2;
new_xyd = [xk(3);xk(4)] + R*[acc_x;acc_y]*dt;
new_xydd = R*[acc_x;acc_y];
new_yaw = (90-(2 + 50/60))/180*pi - atan2(mag_y,mag_x);

weRnotDone = 1;
%this is to correct the new yaw from the magnetometers if we have completed
%1 or more full circles.  Our yaw angle should be able to take any number,
%but the atan2 function only returns numbers between 0 and 2*pi
while weRnotDone
    if new_yaw > theta + 320/180*pi %if yaw~-179 & new_yaw ~ 179
        new_yaw = new_yaw - 2*pi;
    elseif new_yaw < theta - 320/180*pi %if yaw~179 & new_yaw ~ -179
        new_yaw = new_yaw + 2*pi;
    else
        weRnotDone = 0;
    end
end

%now we use the gryoscope measurement for our new yaw rate
new_yaw_dot = gyro;

%now we put all these updated states into our state vector.
x_hat_minus = [new_xy; new_xyd; new_xydd; new_yaw; new_yaw_dot];
end
