function [g, data] = computeBaseBRS(g, tau, twidth, schemeData)
addpath('..')

if nargin < 1
  grid_min = [-5; -5; -pi];
  grid_max = [5; 5; pi];
  N = [51; 51; 51];
  pdDim = 3;
  g = createGrid(grid_min, grid_max, N, pdDim);
end

if nargin < 2
  dt = 0.025;
  tIAT = 2;
  tau = 0:dt:tIAT;
end

if nargin < 3
  twidth = g.dx;
end

data0 = shapeRectangleByCorners(g, -twidth/2, twidth/2);

if nargin < 4
  schemeData.U = [-1 1];
  schemeData.speed = 1;
  schemeData.grid = g;
  schemeData.hamFunc = @dubins3DHamFunc;
  schemeData.partialFunc = @dubins3DPartialFunc;
end

data = HJIPDE_solve(data0, tau, schemeData, 'zero');

end