function obs = genBaseObs0(g, x0, R);
% Generate an ellipse around x0 with axis radii given by R

obs1 = sqrt((1/(R(1)^2))*(g.xs{1} - x0(1)).^2 + (1/(R(2)^2))*(g.xs{2} - x0(2)).^2 +...
  (1/(R(3)^2))*(g.xs{3} - x0(3)).^2) - 1;
obs2 = sqrt((1/(R(1)^2))*(g.xs{1} - x0(1)).^2 + (1/(R(2)^2))*(g.xs{2} - x0(2)).^2 +...
  (1/(R(3)^2))*(g.xs{3} - x0(3) - 2*pi).^2) - 1;
obs = min(obs1, obs2);
