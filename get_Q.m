function Q = get_Q(pos_var, vel_var, acc_var, yawRate_var, mag_var)
%get_Q is a function to get the matrix Q

%Q is a diagonal matrix with the variances we assign.
 Q = diag([pos_var, pos_var, vel_var, vel_var, acc_var, acc_var, mag_var, yawRate_var]);
end
