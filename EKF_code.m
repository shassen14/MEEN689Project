clc;clear all;close all;

%this gets data from my phone
%you need to load the data into Matlab with something like
load('sensorlog_20200410_132333.mat')
%% Acceleration
accDatetime = Acceleration.Timestamp;
[acchrs, accm, accs] = hms(accDatetime);
accTime = accm*60 + accs - (accm(1)*60 + accs(1));
accX = Acceleration.X;
accY = Acceleration.Y;
accZ = Acceleration.Z;

figure(1)
plot(accTime,accX,accTime,accY,accTime,accZ)
title('Acceleration')
legend('X','Y','Z')
grid on

%% Angular Velocity
avDatetime = AngularVelocity.Timestamp;
[avhrs, avm, avs] = hms(avDatetime);
avTime = avm*60 + avs - (avm(1)*60 + avs(1));
avX = AngularVelocity.X;
avY = AngularVelocity.Y;
avZ = AngularVelocity.Z;

figure(2)
plot(avTime,avX,avTime,avY,avTime,avZ)
title('Angular Velocity')
legend('X','Y','Z')
grid on

%% Orientation
oDatetime = Orientation.Timestamp;
[ohrs, om, os] = hms(oDatetime);
oTime = om*60 + os - (om(1)*60 + os(1));
oX = Orientation.X;
oY = Orientation.Y;
oZ = Orientation.Z;

figure(3)
plot(oTime,oX,oTime,oY,oTime,oZ)
title('Orientation')
legend('X','Y','Z')
grid on

%% Magnetic Field
magDatetime = MagneticField.Timestamp;
[maghrs, magm, mags] = hms(magDatetime);
magTime = magm*60 + mags - (magm(1)*60 + mags(1));
magX = MagneticField.X;
magY = MagneticField.Y;
magZ = MagneticField.Z;

figure(4)
plot(magTime,magX,magTime,magY,magTime,magZ)
title('Magnetic Field')
legend('X','Y','Z')
grid on




%x(k) = f(x(k-1))+ w(k-1)
%z(k) = h(x(k))+v(k)
%Structure of the state matrix : 
% [accx]
% [accy]
% [accz]
% [avx]
% [avy]
% [avz]
% [ox]
% [oy]
% [oz]
% [magx]
% [magy]
% [magz]
% 
% 
% 
% 
%


%Optimal Estimate Xo_est = E[xo]

Xo_est = [accX(1),accY(1),accZ(1),avX(1),avY(1),avZ(1),oX(1),oY(1),oZ(1),magX(1),magY(1),magZ(1)]

%Covariance Po

%Po = 













