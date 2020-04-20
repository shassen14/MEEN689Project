clc;clear all;close all;

%% This is some documentation
%this is the state x:
% x = [x
%     y
%     xd
%     yd
%     xdd
%     ydd
%     theta
%     thetad]
%we have 8 states

%% This is Our Real Data!!

mat_file = 'data_collection/walk1_went too far.mat';
load(mat_file)

[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

dt = 0.01;%For now
fastTimes = 0:dt:accel(length(accel),1); %this is the times when our "fast" data comes in.
%This is the data we use in our process model, like accelrometer,
%gyroscope, and magnetometer.
slowTimes = 0:1:gps(length(gps),1) %these are the times when our "slow" data comes
%in.  This is the data we use in our update step, which is our gps position
n = length(fastTimes);%Every other data times
nSlow = length(slowTimes);%GPS times

gps_modified = gps(:,2:3) -gps(1,2:3)

xm = zeros(8,n); %x-hat-minus
xh = zeros(8,n); %x-hat

gps_x = gps_modified(:,1)* 110862.9887;
gps_y = gps_modified(:,2)* 95877.94;

