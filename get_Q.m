function Q = get_Q(acc_var, yawRate_var, mag_var)
%get_Q is a function to get the matrix Q
%  Q = diag([1, 1, 1, 1, 1, 1, 1, 1])*0.02;
 Q = diag([1, 1, 1, 1, acc_var, acc_var, mag_var, yawRate_var]);
end
