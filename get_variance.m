function return_val = get_variance()

% load file
mat_file = 'data_collection/walk2_100.mat';
load(mat_file)

[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

return_val = [var(accel),var(gyro),var(mag_field),var(orientation),var(gps)]

end


