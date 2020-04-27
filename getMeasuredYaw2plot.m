function yaw2plot = getMeasuredYaw2plot(yaw,fastMeasurements)
%getMeasuredYaw2plot uses the yaw angle and the magnetometer measurements
%to find the yaw angle indicated by the magnetometer.  It uses the previous
%yaw angle to know generally where the next yaw angle should be, so that it
%can correct for the limited range of atan2.

mag_x = fastMeasurements(4,:);
mag_y = fastMeasurements(5,:);
new_yaw = (90-(2 + 50/60))/180*pi - atan2(mag_y,mag_x);

for i =1:length(yaw)
weRnotDone = 1;
%this is to correct the new yaw from the magnetometers if we have completed
%1 or more full circles.  Our yaw angle should be able to take any number,
%but the atan2 function only returns numbers between 0 and 2*pi
while weRnotDone
    if new_yaw(i) > yaw(i) + 320/180*pi %if yaw~-179 & new_yaw ~ 179
        new_yaw(i) = new_yaw(i) - 2*pi;
    elseif new_yaw(i) < yaw(i) - 320/180*pi %if yaw~179 & new_yaw ~ -179
        new_yaw(i) = new_yaw(i) + 2*pi;
    else
        weRnotDone = 0;
    end
end
end

yaw2plot = new_yaw;
end