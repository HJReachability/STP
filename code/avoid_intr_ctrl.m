function [u, d] = avoid_intr_ctrl(evader, pursuer, CARS, CARSderiv)
% u = avoid_intr_ctrl(evader, pursuer, CARS, CARSderiv)
%     Computes avoidance control

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

if any(x_rel' >= CARS.g.max) || any(x_rel' <= CARS.g.min)
  u = [0; 0];
else
  p = eval_u(CARS.g, CARSderiv, x_rel);
  u = CARS.dynSys.optCtrl([], x_rel, p, 'max');
  d = CARS.dynSys.optDstb([], x_rel, p, 'min');
end
end