function A = get_A(dt, xk, fastMeasurements)
%get_A is a function to get the matrix A, once it has been linearized
ax_measured = fastMeasurements(1);
ay_measured = fastMeasurements(2);
% mag_x = fastMeasurements(4);
% mag_y = fastMeasurements(5);

% theta = (2+50/60)/180*pi - atan2(mag_y, mag_x);
theta = xk(7);

ax_coeff = ax_measured*-sin(theta) - ay_measured*cos(theta); % glob
ay_coeff = ax_measured*cos(theta) - ay_measured*-sin(theta);

A = [1 0 dt 0 0 0 ax_coeff/2*dt^2 0
    0 1 0 dt 0 0 ay_coeff/2*dt^2 0
    0 0 1 0 0 0 ax_coeff*dt 0
    0 0 0 1 0 0 ay_coeff*dt 0
    0 0 0 0 0 0 ax_coeff 0
    0 0 0 0 0 0 ay_coeff 0
    0 0 0 0 0 0 1 dt
    0 0 0 0 0 0 0 1];
end
