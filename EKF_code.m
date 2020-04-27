clc;clear all;close all;

%% Some Documentation
%{ 
This script reads data from a .mat file, runs the data through an EKF, and
plots the results.  The data was collected from an iPhone XR with the
Matlab Mobile app.  All the data from the phone was collected, including
acclerometer, gyroscope, magnetometer, and GPS data.  This EKF fuses and
filters the data to track the 2D motion and yaw angle of the phone as it
was driven around in a car.  The phone was held level and pointed forward
by the passenger in the car.

The accelerometer, gyroscope, and magnetometer data were collected at 100Hz
while the GPS data was recorded at 1 Hz.

This is the state vector x:
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

%% This is Our Real Data
% Extract the data to arrays
mat_file = 'data_collection/drive3.mat';
load(mat_file)
[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

%now we correct the length of our data, in case some sensors collected one
%more or less data point than the other sensors did.
if length(accel) ~= length(gyro) 
    gyro = gyro(1:end-1,:);
elseif length(accel) ~= length(orientation)
    orientation = orientation(1:end-1,:);
end

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

% zero out the gps coordinates and convert to meters, with our origin
% defined as where we start from.  The global coordinate system is ENU.
gps_y = gps(:,2) * 110862.9887;
gps_x = gps(:,3) * 95877.94;
gps_x = gps_x - gps_x(1);
gps_y = gps_y - gps_y(1);
speed = gps(:,5); %this is speed measurement from gps

%Getting the yaw angle from the magnetometer
yaw = pi*(90-(2+50/60))/180 - atan2(mag_field(:,3), mag_field(:,2));

%using the "orientation" data from the phone, which isn't a unique
%measurement, to rotate the acceleration and account for gravity.
g_body = zeros(3, length(orientation));
for i=1:length(g_body)
    g_body(:,i) = get_RM(pi*orientation(i,2)/180,pi*orientation(i,3)/180, pi*orientation(i,4)/180)'*[0; 0; 9.81];
end

% we group all our fast measurements and all our slow measurements together
% into their own arrays.  We included orientation for debugging.
slow_measurements = [gps_x,gps_y,speed,zeros(size(speed))]';
fast_measurements = [accel(:,2)-g_body(1,:)',accel(:,3)-g_body(2,:)', gyro(:,4), mag_field(:,2), mag_field(:,3), -orientation(:,2)-90]';

% Here are the variances we assume for accelerometer, gyro, and
% magnetometer, and GPS.  
pos_var = 1.5^2;
vel_var = 1.5^2;
acc_var = 1.5^2; 
gyro_var = 1.20^2;
mag_var = 0.00000025^2; % since there is no measurement to update the 
%orientation, the yaw angle estimate always matches the yaw angle
%calculated from the magnetometer reading, so the variance of the
%magnetometer data does not matter.  You can change the number and try it
%out for yourself to see!
gps_var = (5.0/3)^2;
speed_var = 0.5^2;


%% This is our EKF!! :D
% to store all the estimates
xm = zeros(8,n_fast); %x-hat-minus
xh = zeros(8,n_fast); %x-hat

p_init = 1; % this is the number we assume our initial P diagonal to be

x_0 = [0 0 0 0 0 0 atan2(gps_y(10)-gps_y(1),gps_x(10)-gps_x(1)) 0]'; %setting our state's initial condition
P0 = diag([p_init p_init p_init p_init p_init p_init p_init p_init]); %initializing P.  this is a guess
P = zeros(8,8,n_fast); %initializing a variable for all the P(k)
K = zeros(8,4,n_slow); %initializing a variable for all the K(k)

%this is the outline for our EKF:
%do prediction step, 
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

for i = 2:length(fast_times)
    %do prediction step
    fm = fast_measurements(:,i); %this is just an abbreviation 
    xm(:,i) = nonlinear_process(dt_fast(i),xh(:,i-1),fm);
    
    A = get_A(dt_fast(i),xm(:,i),fm);
    Q = get_Q(pos_var, vel_var, acc_var, gyro_var, mag_var);
    
    Pm = A*P(i-1)*A' + Q;
    
    %This condition checks if a new "slow" GPS measurment comes in before
    %the next "fast" measurement, so we know if we have to do the update
    %step.
    if (slowCounter<=n_slow) && (i+1<=n_fast) && (slow_times(slowCounter) < fast_times(i+1))
        %do update step
        H = get_H(xm(7,i));
        R = get_R((gps(slowCounter,7)/5)^2, speed_var);
        
        K(:,:,slowCounter) = (Pm*H')/(H*Pm*H' + R);
        
        xh(:,i) = xm(:,i) + K(:,:,slowCounter)*(slow_measurements(:,slowCounter) - nonlinear_measurement(xm(:,i)));
        P(:,:,i) = (eye(8) - K(:,:,slowCounter)*H)*Pm;
        slowCounter = slowCounter + 1;
    else
        %saving variables if there was no update step
        xh(:,i) = xm(:,i);
        P(:,:,i) = Pm;
    end
end

%% plots
figure(1)
plot(xh(1,:),xh(2,:),'-b',gps_x,gps_y,'og')
title('position')
legend('estimated','gps measured')
grid on; axis equal

%adjusting the yaw angle to account for the limited range of atan2
for i =2:length(yaw)
    if  yaw(i) > yaw(i-1) + 320/180*pi %if yaw~-179 & new_yaw ~ 179
        yaw(i) = yaw(i) - 2*pi;
    elseif yaw(i) < yaw(i-1) - 320/180*pi %if yaw~179 & new_yaw ~ -179
        yaw(i) = yaw(i) + 2*pi;
    end
end

figure(2)
plot(fast_times,yaw/pi*180,'-b')
title('Yaw Angle')
legend('Estimated from Magnetometer')
ylabel('Angle (deg)')
xlabel('Time (s)')
grid on; 

%rotating the velocity into the local sensor frame, so we can plot it
vel_loc_x = cos(xh(7,:)).*xh(3,:) + sin(xh(7,:)).*xh(4,:);
vel_loc_y = -sin(xh(7,:)).*xh(3,:) + cos(xh(7,:)).*xh(4,:);

figure(3)
plot(fast_times,vel_loc_x,fast_times, vel_loc_y)
title('Local Velocity')
legend('velocity x', 'velocity y')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
grid on; 

figure(4)
plot(fast_times,xh(3,:),fast_times, xh(4,:))
title('Global Velocity')
legend('Velocity X', 'Velocity Y')
ylabel('Velocity (m/s)')
xlabel('Time (s)')
grid on; 

figure(5)
plot(fast_times,fast_measurements(1,:),fast_times, fast_measurements(2,:))
title('Local Acceleration')
legend('acceleration x', 'acceleration y')
ylabel('Acceleration (m/s^2)')
xlabel('Time (s)')
grid on; 

figure(6)
plot(fast_times,xh(5,:),fast_times, xh(6,:))
title('Global Acceleration')
legend('Acceleration X', 'Acceleration Y')
ylabel('Acceleration (m/s^2)')
xlabel('Time (s)')
grid on; 

figure(7)
plot(slow_times, speed, fast_times, sqrt(xh(3,:).^2 + xh(4,:).^2))
title('Velocity Magnitude')
legend('GPS Speed', 'Estimated Speed')
ylabel('Speed (m/s)')
xlabel('Time (s)')
grid on; 

