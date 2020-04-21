clc;clear all;close all;
%% This is some documentation
%{ 
this is the state x:
x = [x
    y
    xd
    yd
    xdd
    ydd
    theta
    thetad]
we have 8 states
%}

%% This is Our Real Data!!
% Extract the data to arrays
mat_file = 'data_collection/walk1_went too far.mat';
load(mat_file)
[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

% create dt as an array of the fast data (accel, gyro, mag) for the process
% model update. This is assuming all fast data are synced time wise
% I noticed fastTimes had more rows than the data collected because at some
% time steps, the measurements were a little slow (i.e dt = .0105 or .0110)
dt_fast = gradient(accel(:,1)); 

% create dt as an array of slow data (gps) for the measurement update
dt_slow = gradient(gps(:,1));

% create times for the fast and slow data
fast_times = accel(:,1);
slow_times = gps(:,1);

n_fast = length(fast_times);
n_slow = length(slow_times);

% zero out the gps coordinates and convert to utm coordinates
% [lat, long] = [y,x]
% gps_modified = gps(:,2:3) - gps(1,2:3);
% gps_y = gps_modified(:,1) * 110862.9887;
% gps_x = gps_modified(:,2) * 95877.94;

gps_y = gps(:,2) * 110862.9887;
gps_x = gps(:,3) * 95877.94;
gps_x = gps_x - gps_x(1);
gps_y = gps_y - gps_y(1);

yaw = 0*pi*(2+50/60)/180 + atan2(mag_field(:,3), mag_field(:,2));
% q = ecompass(accel(:,2:4),mag_field(:,2:4));
% e = eulerd(q,'ZYX','frame');
% e1 = e(:,1)-90

% have fast and slow measurements in one array. I've added orientation for
% debugging purposes
slow_measurements = [gps_x,gps_y]';
fast_measurements = [accel(:,2),accel(:,3), gyro(:,4), mag_field(:,2), mag_field(:,3), orientation(:,2)]';

% Here are the variances we assume for accelerometer, gyro, and
% magnetometer, and gps. ToDo: change the variances 
acc_var = 10.0^2; % std of accelerometer is 0.125 m/s^2
gyro_var = 1.20^2;
mag_var = 10.0^2;
gps_var = 0.5^2;


%% This is our EKF!! :D
% to store all the estimates
xm = zeros(8,n_fast); %x-hat-minus
xh = zeros(8,n_fast); %x-hat

p_init = 1; % this is the number we assume our initial P diagonal to be

x_0 = [0 0 0 1 0 0 atan2(gps_y(4)-gps_y(1),gps_x(4)-gps_x(1)) 1]'; %setting our state's initial condition
P0 = diag([p_init p_init p_init p_init p_init p_init p_init p_init]); %initializing P.  this is a guess
P = zeros(8,8,n_fast); %initializing a variable for all the P(k)
K = zeros(8,2,n_slow); %initializing a variable for all the K(k)

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
slowCounter = 4; %this is the counter to tell what "slow" measurement we 
%are using.

%FYI, there is a problem with the EKF not going to the last data point, but
%I don't think that's important right nw.
for i = 2:length(fast_times)-1000
    %do prediction step
    fm = fast_measurements(:,1); %this is just an abbreviation 
    xm(:,i) = nonlinear_process(dt_fast(i),xh(:,i-1),fast_measurements(:,i));
    Pm = get_A(dt_fast(i),xm(:,i),fm)*P(i-1)*get_A(dt_fast(i),xm(:,i),fm)' + get_Q(acc_var, gyro_var, mag_var);
    if slow_times(slowCounter) < fast_times(i+1)
        %do update step
        K(:,:,slowCounter) = (Pm*get_H()')/(get_H()*Pm*get_H()' + get_R(sqrt(gps_var)*3));
        xh(:,i) = xm(:,i) + K(:,:,slowCounter)*(slow_measurements(:,slowCounter) - nonlinear_measurement(xm(:,i)));
        P(:,:,i) = (eye(8) - K(:,:,slowCounter)*get_H())*Pm;
        slowCounter = slowCounter + 1;
    else
        xh(:,i) = xm(:,i);
        P(:,:,i) = Pm;
    end
end

%% plots
figure(1)
plot(xh(1,:),xh(2,:),'-ob',gps_x,gps_y,'og')
title('position')
legend('estimated','gps measured')
grid on; axis equal

figure(12)
plot(orientation(:,1), orientation(:,2))

figure(13)
plot(mag_field(:,2),mag_field(:,4))

figure(14)
plot(gyro(:,1), gyro(:,4))

figure(2)
yaw2plot = getMeasuredYaw2plot(yaw,fast_measurements);
plot(fast_times,xh(7,:),'b',fast_times,yaw2plot,'--g', fast_times, pi*orientation(:,2)/180)
title('Yaw Angle')
legend('estimated','measured from mag', 'orientation reading')
grid on; axis equal

figure(3)
plot(fast_times,xh(8,:),'-b')
title('yaw rate')
legend('measured')
grid on;

%% These are functions to get the matrices and vectors we use




