clear all; close all; clc;

% This code is to calculate standard deviation for sensor accuracy.

% load file
mat_file = 'MobileSensorData/IphoneSensorAccuracy_RoomFloor';
load(mat_file)

% data is extracted properly into rows and columns
[accel, gyro, mag_field] = dataExtract_SensorAccuracy(mat_file);

% take mean and standard deviation accelerometer
%{
mean_acc_X = mean(accel(:,2));
mean_acc_Y = mean(accel(:,3));
mean_acc_Z = mean(accel(:,4)); 
%}
std_acc_X = std(accel(:,2));
std_acc_Y = std(accel(:,3));
std_acc_Z = std(accel(:,4));

% take mean and standard deviation gyroscope
%{
mean_gy_X = mean(gyro(:,2));
mean_gy_Y = mean(gyro(:,3));
mean_gy_Z = mean(gyro(:,4)); 
%} 
std_gy_X = std(gyro(:,2));
std_gy_Y = std(gyro(:,3));
std_gy_Z = std(gyro(:,4));

% take mean and standard deviation magnetometer
%{
mean_mag_X = mean(mag_field(:,2));
mean_mag_Y = mean(mag_field(:,3));
mean_mag_Z = mean(mag_field(:,4));
%}
std_mag_X = std(mag_field(:,2));
std_mag_Y = std(mag_field(:,3));
std_mag_Z = std(mag_field(:,4));

