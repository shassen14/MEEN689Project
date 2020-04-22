function R = get_RM(y,p,r)
R = [cos(r)*cos(y) - sin(p)*sin(r)*sin(y), -cos(p)*sin(y), cos(y)*sin(r) + cos(r)*sin(p)*sin(y);
    cos(r)*sin(y) + cos(y)*sin(p)*sin(r), cos(p)*cos(y), sin(r)*sin(y) - cos(r)*cos(y)*sin(p);
    -cos(p)*sin(r), sin(p), cos(p)*cos(r)];
end