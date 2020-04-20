clc;clear all;close all;

% This is some documentation
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

% This is Our Real Data!!

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

slow_y = [gps_x,gps_y]';%ToDo change slow_y to slow_measurements
fastMeasurements = [accel(:,2),accel(:,3), gyro(:,4), mag_field(:,2), mag_field(:,3)]';

%% This is our EKF!! :D
x_0 = [0 0 0 1 -1 0 0 1]'; %setting our state's initial condition
P0 = diag([1 1 1 1 1 1 1 1]); %initializing P.  this is a guess
P = zeros(8,8,n); %initializing a variable for all the P(k)
K = zeros(8,2,nSlow); %initializing a variable for all the K(k)

%this is the outline for our EKF
%do prediction step, with fixed dt, 
%check if we have a measurment step
%   by checking if next measurement time is before next 
%   prediction time
%if not, save xm to xh.
%if so, save xh to xh.
%then redo

xh(:,1) = x_0; %setting the initial value for x-hat and P
P(:,:,1) = P0;
slowCounter = 1; %this is the counter to tell what "slow" measurement we 
%are using.

%FYI, there is a problem with the EKF not going to the last data point, but
%I don't think that's important right now.
for i = 2:length(fastTimes)-1
    %do prediction step
    fm = fastMeasurements(:,1); %this is just an abbreviation 
    xm(:,i) = nonlinear_process(dt,xh(:,i-1),fastMeasurements(:,i));
    Pm = get_A(dt,xm(:,i),fm)*P(i-1)*get_A(dt,xm(:,i),fm)' + get_Q(acc_var, yawRate_var, mag_var);
    if slowTimes(slowCounter) < fastTimes(i+1)
        %do update step
        K(:,:,slowCounter) = (Pm*get_H()')/(get_H()*Pm*get_H()' + get_R(sqrt(gps_var)*3));
        xh(:,i) = xm(:,i) + K(:,:,slowCounter)*(slow_y(:,slowCounter) - nonlinear_measurement(xm(:,i)));
        P(:,:,i) = (eye(8) - K(:,:,slowCounter)*get_H())*Pm;
        slowCounter = slowCounter + 1;
    else
        xh(:,i) = xm(:,i);
        P(:,:,i) = Pm;
    end
end



%% These are functions to get the matrices and vectors we use
function A = get_A(dt, xk, fastMeasurements)
%get_A is a function to get the matrix A, once it has been linearized
theta = xk(7);
ax_measured = fastMeasurements(1);
ay_measured = fastMeasurements(2);
% if theta ~= 0
%     ax_coeff = (ax_measured*cos(theta) - ay_measured*sin(theta))/theta;
%     ay_coeff = (ax_measured*sin(theta) + ay_measured*cos(theta))/theta;
% else
%     ax_coeff = ax_measured;
%     ay_coeff = ay_measured;
% end

ax_coeff = ax_measured*-sin(theta) - ay_measured*cos(theta);
ay_coeff = ax_measured*cos(theta) - ay_measured*-sin(theta);

A = [1 0 dt 0 0 0 ax_coeff/2*dt^2 0
    0 1 0 dt 0 0 ay_coeff/2*dt^2 0
    0 0 1 0 0 0 ax_coeff*dt 0
    0 0 0 1 0 0 ay_coeff*dt 0
    0 0 0 0 0 0 ax_coeff 0
    0 0 0 0 0 0 ay_coeff 0
    0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0];
end


function x_hat_minus = nonlinear_process(dt, xk, fastMeasurements)
%nonlinear_process is the nonlinear x=A(x) process model
%new_state = zeros(8,1);
theta = xk(7);
acc_x = fastMeasurements(1);
acc_y = fastMeasurements(2);
gyro = fastMeasurements(3);
mag_x = fastMeasurements(4);
mag_y = fastMeasurements(5);

R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];

new_xy = [xk(1);xk(2)] + [xk(3);xk(4)]*dt + R*[acc_x;acc_y]/2*dt^2;
new_xyd = [xk(3);xk(4)] + R*[acc_x;acc_y]*dt;
new_xydd = R*[acc_x;acc_y];
new_yaw = (2 + 50/60)/180*pi - atan2(mag_y,mag_x);

weRnotOK = 1;
%this is to correct the new yaw from the magnetometers if we have completed
%1 or more full circles.  Our yaw angle should be able to take any number,
%but the atan2 function only returns numbers between 0 and 2*pi
while weRnotOK
    if new_yaw > theta + 320/180*pi %if yaw~-179 & new_yaw ~ 179
        new_yaw = new_yaw - 2*pi;
    elseif new_yaw < theta - 320/180*pi %if yaw~179 & new_yaw ~ -179
        new_yaw = new_yaw + 2*pi;
    else
        weRnotOK = 0;
    end
end

%we could have a problem here about 361->1 and so on
new_yaw_dot = gyro;
new_state = [new_xy; new_xyd; new_xydd; new_yaw; new_yaw_dot];
x_hat_minus = new_state;

end



function H = get_H()
%get_H is a function to get the vector H
%lat2meters = 110862.9887;
%long2meters = 95877.94;
%H = [1/lat2meters, 0, 0, 0, 0, 0, 0, 0;
%    0, 1/long2meters, 0, 0, 0, 0, 0, 0];
%since our GPS uncertainty is given in meters, I think we should say our
%measurement is also in meters - having such small numbers in H might cause
%problems, but idk - we should probably try it both ways
H = [1, 0, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0, 0];
end


function R = get_R(h_accuracy)
%get R is a function to get the matrix R
gps_variance = (h_accuracy/3)^2; %I'm assuming the "accuracy" it gives us 
%is 3 times the standard deviation, but this is just a guess.

R = diag([gps_variance, gps_variance]); %maybe instead of a diagonal
%matrix, all the elements should be the same, since the latitude and
%longitude variances are related?
end


function Q = get_Q(acc_var, yawRate_var, mag_var)
%get_Q is a function to get the matrix Q
 Q = diag([1, 1, 1, 1, 1, 1, 1, 1])*0.02;
end