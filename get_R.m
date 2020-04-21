function R = get_R(h_accuracy)
%get R is a function to get the matrix R
gps_variance = (h_accuracy)^2; %I'm assuming the "accuracy" it gives us 
%is 3 times the standard deviation, but this is just a guess.

R = diag([gps_variance, gps_variance]); %maybe instead of a diagonal
%matrix, all the elements should be the same, since the latitude and
%longitude variances are related?
end
