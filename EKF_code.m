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
mat_file = 'data_collection/drive3.mat';
load(mat_file)
[accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file);

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

% zero out the gps coordinates and convert to utm coordinates
% [lat, long] = [y,x]
% gps_modified = gps(:,2:3) - gps(1,2:3);
% gps_y = gps_modified(:,1) * 110862.9887;
% gps_x = gps_modified(:,2) * 95877.94;

gps_y = gps(:,2) * 110862.9887;
gps_x = gps(:,3) * 95877.94;
gps_x = gps_x - gps_x(1);
gps_y = gps_y - gps_y(1);
speed = gps(:,5); %this is speed measurement from gps

yaw = pi*(90-(2+50/60))/180 - atan2(mag_field(:,3), mag_field(:,2));
% q = ecompass(accel(:,2:4),mag_field(:,2:4));
% e = eulerd(q,'ZYX','frame');
% e1 = e(:,1)-90

g_body = zeros(3, length(orientation));
for i=1:length(g_body)
    g_body(:,i) = get_RM(pi*orientation(i,2)/180,pi*orientation(i,3)/180, pi*orientation(i,4)/180)'*[0; 0; 9.81];
end

g_body(:,i) = get_RM(pi*orientation(i,2)/180,pi*orientation(i,3)/180, pi*orientation(i,4)/180)'*[0; 0; 9.81];


% have fast and slow measurements in one array. I've added orientation for
% debugging purposes
slow_measurements = [gps_x,gps_y,speed,zeros(size(speed))]';
fast_measurements = [accel(:,2)-g_body(1,:)',accel(:,3)-g_body(2,:)', gyro(:,4), mag_field(:,2), mag_field(:,3), -orientation(:,2)-90]';

% Here are the variances we assume for accelerometer, gyro, and
% magnetometer, and gps. ToDo: change the variances 
pos_var = 1.5^2;
vel_var = 1.5^2;
acc_var = 1.5^2; % std of accelerometer is 0.125 m/s^2
gyro_var = 1.20^2;
mag_var = 0.00000025^2;
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
%I don't think that's important right nw.
for i = 2:length(fast_times)
    %do prediction step
    fm = fast_measurements(:,1); %this is just an abbreviation 
    xm(:,i) = nonlinear_process(dt_fast(i),xh(:,i-1),fast_measurements(:,i));
    Pm = get_A(dt_fast(i),xm(:,i),fm)*P(i-1)*get_A(dt_fast(i),xm(:,i),fm)' + get_Q(pos_var, vel_var, acc_var, gyro_var, mag_var);
    if (slowCounter<=n_slow) && (i+1<=n_fast) && (slow_times(slowCounter) < fast_times(i+1))
        %do update step
        K(:,:,slowCounter) = (Pm*get_H(xm(7,i))')/(get_H(xm(7,i))*Pm*get_H(xm(7,i))' + get_R((gps(slowCounter,7)/5)^2, speed_var));
        xh(:,i) = xm(:,i) + K(:,:,slowCounter)*(slow_measurements(:,slowCounter) - nonlinear_measurement(xm(:,i)));
        P(:,:,i) = (eye(8) - K(:,:,slowCounter)*get_H(xm(7,i)))*Pm;
        slowCounter = slowCounter + 1;
    else
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


% figure(2)
% yaw2plot = getMeasuredYaw2plot(yaw,fast_measurements);
% plot(fast_times,xh(7,:),'b',fast_times,yaw2plot/pi*180,'--g', fast_times, orientation(:,2))
% title('Yaw Angle')
% legend('estimated','measured from mag', 'orientation reading')
% grid on; 
% 
% figure(2)
% plot(fast_times,mod(xh(7,:)/pi*180+360,360),'b',fast_times,mod(yaw/pi*180+360,360),'--g', fast_times, mod(orientation(:,2)+360,360))
% title('Yaw Angle')
% legend('estimated','measured from mag', 'orientation reading')
% grid on;


figure(2)
plot(fast_times,yaw/pi*180,'-b')
title('Yaw Angle')
legend('Estimated from Magnetometer')
ylabel('Angle (rad)')
xlabel('Time (s)')
grid on; 

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

% figure(8)
% boxplot(gps(:,7))
% grid on;

% figure(9)
% plot(mag_field(:,2),mag_field(:,3))
% title('Mag X vs Mag Y')
% grid on; axis equal



% figure(10);
% hold on
% plot1 = scatter(gps_x(1),gps_y(1),'og');
% plot2 = plot(xh(1,1), xh(2,1),'-b');
% xlim([min(gps_x)-20 max(gps_x)+20]);
% ylim([min(gps_y)-20 max(gps_y)+20]);
% xlabel('Global X (m)')
% ylabel('Global Y (m)')
% title('Position')
% % set(gca,'Color','none');
% % set(gca,'CLim',[0, 1E-4]);
% grid on;
% 
% count = 1;
% for k = 2:length(xh) 
%     plot2.XData = xh(1,1:k);
%     plot2.YData = xh(2,1:k);
%     if (count<=n_slow) && (k+1<=n_fast) && (slow_times(count) < fast_times(k+1))
%         plot1.XData = gps_x(1:count);
%         plot1.YData = gps_y(1:count);
%         count = count + 1;
%     end
%     % pause 2/10 second:
%     pause(0.004)
% end
%% These are functions to get the matrices and vectors we use




