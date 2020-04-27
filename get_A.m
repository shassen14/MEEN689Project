function A = get_A(dt, xk, fastMeasurements)
%get_A is a function to get the matrix A, once it has been linearized.

%we use the measured accelerations in the process model
ax_measured = fastMeasurements(1);
ay_measured = fastMeasurements(2);

%getting the yaw angle so we can rotate the measurements into the inertial
%frame
theta = xk(7);

%rotating mesasurements into the global frame
ax_coeff = ax_measured*-sin(theta) - ay_measured*cos(theta); 
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
