function H = get_H(theta)
%get_H is a function to get the vector H

%this H vector uses theta so that we can rotate the measured GPS speed into
%the body frame.
H = [1, 0, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0, 0
    0, 0, -sin(theta), cos(theta), 0, 0, 0, 0
    0, 0, cos(theta), sin(theta), 0, 0, 0, 0];
end