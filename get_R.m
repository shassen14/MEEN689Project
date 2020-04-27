function R = get_R(gps_variance, speed_var)
%get R is a function to get the matrix R

%R is a diagonal matrix with the variances we assign to the GPS sensor data
R = diag([gps_variance, gps_variance, speed_var, speed_var]); 
end
