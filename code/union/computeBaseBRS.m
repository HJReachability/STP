function [g, data] = computeBaseBRS(tau, twidth, schemeData)
addpath('..')

if nargin < 1
  dt = 0.025;
  tIAT = 2;
  tau = 0:dt:tIAT;
end

if nargin < 3
  twidth = g.dx;
end

g = schemeData.grid;
data0 = shapeRectangleByCorners(schemeData.grid, -twidth/2, twidth/2);

if nargin < 3
  schemeData.U = [-1 1];
  schemeData.speed = 1;
  schemeData.grid = g;
  schemeData.hamFunc = @dubins3DHamFunc;
  schemeData.partialFunc = @dubins3DPartialFunc;
end

data = HJIPDE_solve(data0, tau, schemeData, 'zero');

end