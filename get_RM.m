function R = get_RM(y,p,r)
%get_RM is a function that provides a rotation matrix to rotate a vector
%from the body frame to the inertial frame.  This function requires yaw,
%pitch, and roll as scalars.

%The rotation matrix depends on roll, pitch, and yaw.
R = [cos(r)*cos(y) - sin(p)*sin(r)*sin(y), -cos(p)*sin(y), cos(y)*sin(r) + cos(r)*sin(p)*sin(y);
    cos(r)*sin(y) + cos(y)*sin(p)*sin(r), cos(p)*cos(y), sin(r)*sin(y) - cos(r)*cos(y)*sin(p);
    -cos(p)*sin(r), sin(p), cos(p)*cos(r)];
end