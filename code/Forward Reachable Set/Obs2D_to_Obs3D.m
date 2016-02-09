function data3D = Obs2D_to_Obs3D(data2D, g3D)
% data3D = Obs2D_to_Obs3D(data2D, g3D)
%
% To test this function, run it without any input parameters

%% Create test shape (circle of random radius less than 1)
if nargin < 1
  g3D.dim = 3;
  g3D.min = [-1; -1; 0];
  g3D.max = [1; 1; 2*pi];
  g3D.N = 51;
  g3D.bdry = {@addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
  g3D = processGrid(g3D);
  
  g2D = proj2D(g3D, [], [0 0 1]);
  data2D = shapeSphere(g2D, 0, rand); 
end

%% Compute 3D data and plot
data3D = repmat(data2D, [1 1 g3D.N(3)]);

if nargin < 1
  figure;
  contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'b', 'linewidth', 5)
  hold on
  
  visualizeLevelSet(g3D, data3D, 'surface', 0);
  camlight left
  camlight right
end
end