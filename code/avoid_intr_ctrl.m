function [u, d] = avoid_intr_ctrl(evader, pursuer, CARS, CARS2)
% u = avoid_intr_ctrl(evader, pursuer, CARS, CARSderiv)
%     Computes avoidance control

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

if any(x_rel(1:2) >= CARS.g.max(1:2)) || ...
    any(x_rel(1:2) <= CARS.g.min(1:2))
%   if nargin < 4
%     u = [0; 0];
%     d = zeros(5,1);
%     d(1) = max(pursuer.vrange);
%   else
% fprintf("Using CARS2\n")
    p = eval_u(CARS2.g, CARS2.deriv, x_rel);
    u = CARS2.dynSys.optCtrl([], x_rel, p, 'max');
    d = CARS2.dynSys.optDstb([], x_rel, p, 'min');    
%   end
else
%   fprintf("Using CARS\n")
  p = eval_u(CARS.g, CARS.deriv, x_rel);
  u = CARS.dynSys.optCtrl([], x_rel, p, 'max');
  d = CARS.dynSys.optDstb([], x_rel, p, 'min');
end
end