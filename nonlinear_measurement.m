function cx = nonlinear_measurement(xk)
%This is the nonlinear version of z=Hx
cx = get_H(xk(7))*xk;
end
