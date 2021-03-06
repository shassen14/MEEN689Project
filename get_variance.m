function return_val = get_variance()
%get_variance returns the variance of different sensor readings while the
%phone was sitting still.

% load file
mat_file = 'data_collection/sensorlog_Variance_AkshayPhone.mat';

[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

return_val = [var(accel),var(gyro),var(mag_field),var(orientation),var(gps)];

end


