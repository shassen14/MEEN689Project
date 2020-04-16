clear all; clc; close all;

% load file
mat_file = 'data_collection/walk2_100.mat';
load(mat_file)

[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);



