function H = get_H(theta)
%get_H is a function to get the vector H
%lat2meters = 110862.9887;
%long2meters = 95877.94;
%H = [1/lat2meters, 0, 0, 0, 0, 0, 0, 0;
%    0, 1/long2meters, 0, 0, 0, 0, 0, 0];
%since our GPS uncertainty is given in meters, I think we should say our
%measurement is also in meters - having such small numbers in H might cause
%problems, but idk - we should probably try it both ways
H = [1, 0, 0, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0, 0, 0
    0, 0, -sin(theta), cos(theta), 0, 0, 0, 0
    0, 0, cos(theta), sin(theta), 0, 0, 0, 0];
end