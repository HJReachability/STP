function [u1, u2, safety_val] = evadeControl(obj, other, PE_BRS)
% [u, safe] = evadeControl(PE_BRS, xr, threshold)
% 
%

% Safety value
xr = obj.getRelStates(other, 'pi');
safety_val = eval_u(PE_BRS.g, PE_BRS.data, xr);

% Gradient and determinants
p = eval_u(PE_BRS.g, PE_BRS.grad, xr);

det_v1 = -p(1);
det_v2 = p(1)*cos(xr(3)}) + p(2)*sin(xr(3));
det_w1 = p(1)*xr(2) - p(2).*xr(1) - p(3);
det_w2 = p(3);

% Control
v1 = (det_v1>=0)*obj.vrange(1) + (det_v1<0)*obj.vrange(2);

end