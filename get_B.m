function B = get_B(mag_x, mag_y, gyro_z)
%get_B is a function to get the vector B.  We might not use it, but if we
%did, this is what it would be.

theta_measured = (2+50/60)/180*pi - atan2(mag_y, mag_x); %this is in radians
%we might have a problem if 361->1, but we can correct this later
theta_dot_measured = gyro_z;

B = [0, 0, 0, 0, 0, 0, theta_measured, theta_dot_measured]';
end