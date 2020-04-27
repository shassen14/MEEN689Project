function z = nonlinear_measurement(xk)
%nonlinear_measurement is the nonlinear version of z=Hx.  For now, this
%function is linear, but if it was nonlinear, this is where we would put
%the nonlinear measurement model.
z = get_H(xk(7))*xk;
end
