function augment_staticObs_intr2(obj, save_png)
% augment_staticObs_intr2(obj, save_png)

if nargin < 2
  save_png = true;
end

fprintf('Executing %s...\n', mfilename)

if save_png
  if ispc
    folder = sprintf('%s\\%s', obj.folder, mfilename);
  else
    folder = sprintf('%s/%s', obj.folder, mfilename);
  end
  system(sprintf('mkdir %s', folder));
end

%% Augment by tracking radius
augObs2D = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
augObs3D = repmat(augObs2D, [1 1 obj.gN(3)]);

%% Compute BRS from tracking-radius-augmented obstacles
dynSys = Plane(zeros(3,1), obj.wMaxA, obj.vRangeA, obj.dMaxA);
sD.grid = obj.g;
sD.uMode = 'min';
sD.dMode = 'min';
sD.dynSys = dynSys;

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.fig_filename = sprintf('%s/', folder);
extraArgs.keepLast = true;

augStaticObs = HJIPDE_solve(augObs3D, [0 obj.tIAT], sD, 'zero', extraArgs);

%% Fill obstacle throughout time
% Boundary obstacle
obs_bdry = -shapeRectangleByCorners(obj.g, obj.g.min + [5; 5; -inf], ...
  obj.g.max - [5; 5; -inf]);
obj.augStaticObs = min(augStaticObs, obs_bdry);
obj.augStaticObs = repmat(obj.augStaticObs, [1 1 1 length(obj.tau)]);

end