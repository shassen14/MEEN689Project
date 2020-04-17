clear all; clc; close all;
% this is strictly just to test code that you want. Go crazy


% load file
mat_file = 'data_collection/walk2_100.mat';
load(mat_file)

[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);