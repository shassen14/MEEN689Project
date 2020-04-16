function [accel, gyro, mag_field, orientation, gps] = dataExtract(mat_file)
%DATA_EXTRACT is to extract mat files into arrays
%   1st column is timestamp, 2 - n columns are the data (i.e. x, y, z, etc)

% load mat file data
load(mat_file, 'Acceleration', 'AngularVelocity', 'MagneticField', 'Orientation', 'Position');

%% Acceleration Data
accDatetime = Acceleration.Timestamp;
[acchrs, accm, accs] = hms(accDatetime);
accTime = accm*60 + accs - (accm(1)*60 + accs(1));
accX = Acceleration.X;
accY = Acceleration.Y;
accZ = Acceleration.Z;

accel = [accTime, accX, accY, accZ];


%% Gyroscope Data
avDatetime = AngularVelocity.Timestamp;
[avhrs, avm, avs] = hms(avDatetime);
avTime = avm*60 + avs - (avm(1)*60 + avs(1));
avX = AngularVelocity.X;
avY = AngularVelocity.Y;
avZ = AngularVelocity.Z;

gyro = [avTime, avX, avY, avZ];


%% Magnetic Field Data
magDatetime = MagneticField.Timestamp;
[maghrs, magm, mags] = hms(magDatetime);
magTime = magm*60 + mags - (magm(1)*60 + mags(1));
magX = MagneticField.X;
magY = MagneticField.Y;
magZ = MagneticField.Z;

mag_field = [magTime, magX, magY, magZ];


%% Orientation Data
oDatetime = Orientation.Timestamp;
[ohrs, om, os] = hms(oDatetime);
oTime = om*60 + os - (om(1)*60 + os(1));
oX = Orientation.X;
oY = Orientation.Y;
oZ = Orientation.Z;

orientation = [oTime, oX, oY, oZ];


%% GPS Data
gpsDatetime = Position.Timestamp;
[gpshrs, gpsm, gpss] = hms(gpsDatetime);
gpsTime = gpsm*60 + gpss - (gpsm(1)*60 + gpss(1));
gpsX = Position.latitude;
gpsY = Position.longitude;
gpsZ = Position.altitude;
gpsSpeed = Position.speed;
gpsCourse = Position.course;
gpsHacc = Position.hacc;

% [Time, lat, long, altitude, speed, heading, horizontal accuracy]
gps = [gpsTime, gpsX, gpsY, gpsZ, gpsSpeed, gpsCourse, gpsHacc];


end

