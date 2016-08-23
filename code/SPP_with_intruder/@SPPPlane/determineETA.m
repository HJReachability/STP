function determineETA(obj, tauFRS, schemeData, obstacles)
% ETA = determineETA(vehicle, tauFRS, schemeData, obstacles)
%     Computes the ETA of a vehicle

schemeData.uMode = 'max';
schemeData.dMode = 'min';
schemeData.tMode = 'forward';

% Center the grid between target and current vehicle state
center = 0.5*(obj.targetCenter + obj.x);

new_gmin = center - 1;
new_gmin(3) = schemeData.grid.min(3); 
new_gmax = center + 1;
new_gmax(3) = schemeData.grid.max(3);
new_g = createGrid(new_gmin, new_gmax, schemeData.grid.N, 3);
old_g = schemeData.grid;
schemeData.grid = new_g;

% Min with target
extraArgs.targets = shapeEllipsoid(schemeData.grid, obj.x, ...
  2*schemeData.grid.dx);

% Set obstacles
extraArgs.obstacles = zeros(size(obstacles));
for i = 1:size(obstacles, 4)
  extraArgs.obstacles(:,:,:,i) = ...
    migrateGrid(old_g, obstacles(:,:,:,i), schemeData.grid);
end

% Computation should stop once it contains the initial state
extraArgs.stopSetIntersect = shapeCylinder(schemeData.grid, 3, ...
  obj.data.targetCenter, obj.data.targetRsmall);

% Compute the FRS
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = obj.x(3);

folder = sprintf('FRS_%f', now);
system(sprintf('mkdir %s', folder));

extraArgs.fig_filename = sprintf('%s/', folder);
[obj.data.FRS1, obj.data.FRS1_tau] = ...
  HJIPDE_solve(extraArgs.targets, tauFRS, schemeData, 'none', extraArgs);

obj.data.FRS1_g = schemeData.grid;

% Extract ETA
obj.data.ETA = obj.data.FRS1_tau(end);
end