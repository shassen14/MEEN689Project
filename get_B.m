function B = get_B(mag_x, mag_y, gyro_z)
%get_B is a function to get the vector B
theta_measured = (2+50/60)/180*pi - atan2(mag_y, mag_x); %in radians
%might have a problem if 361->1
theta_dot_measured = gyro_z;

B = [0, 0, 0, 0, 0, 0, theta_measured, theta_dot_measured]';
end