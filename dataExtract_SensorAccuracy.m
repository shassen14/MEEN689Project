function [accel, gyro, mag_field] = dataExtract(mat_file)
%DATA_EXTRACT_SensorAccuracy is to extract phone .mat files into arrays
%   1st column is timestamp, 2 - n columns are the data (i.e. x, y, z, etc)
%   All sensors are in body frame

% load mat file data
load(mat_file, 'Acceleration', 'AngularVelocity', 'MagneticField');

%% Acceleration Data
accDatetime = Acceleration.Timestamp;
[acchrs, accm, accs] = hms(accDatetime);
accTime = accm*60 + accs - (accm(1)*60 + accs(1));
accX = Acceleration.X;
accY = Acceleration.Y;
accZ = Acceleration.Z;

% [time, Acceleriation in X direction, Acceleration in Y direction,
% Accleration Z direction] 
accel = [accTime, accX, accY, accZ];


%% Gyroscope Data
avDatetime = AngularVelocity.Timestamp;
[avhrs, avm, avs] = hms(avDatetime);
avTime = avm*60 + avs - (avm(1)*60 + avs(1));
avX = AngularVelocity.X;
avY = AngularVelocity.Y;
avZ = AngularVelocity.Z;

% [time, Pitch rate, Roll rate, Yaw rate]
gyro = [avTime, avX, avY, avZ];


%% Magnetic Field Data
magDatetime = MagneticField.Timestamp;
[maghrs, magm, mags] = hms(magDatetime);
magTime = magm*60 + mags - (magm(1)*60 + mags(1));
magX = MagneticField.X;
magY = MagneticField.Y;
magZ = MagneticField.Z;

% [time, magnetic field x direction, mag field y direction, mag field z
% direction]
mag_field = [magTime, magX, magY, magZ];

end

