function vehicle = determineETA(vehicle, tauFRS, schemeData, obstacles)
% ETA = determineETA(vehicle, tauFRS, schemeData, obstacles)
%     Computes the ETA of a vehicle

schemeData.uMode = 'max';
schemeData.dMode = 'min';
schemeData.tMode = 'forward';

% Center the grid between target and current vehicle state
center = 0.5*(vehicle.data.targetCenter + vehicle.x);

new_gmin = center - 0.5;
new_gmin(3) = schemeData.grid.min(3); 
new_gmax = center + 0.5;
new_gmax(3) = schemeData.grid.max(3);
new_g = createGrid(new_gmin, new_gmax, schemeData.grid.N, 3);
old_g = schemeData.grid;
schemeData.grid = new_g;

% Min with target
extraArgs.targets = shapeEllipsoid(schemeData.grid, vehicle.x, ...
  2.5*schemeData.grid.dx);

% Set obstacles
extraArgs.obstacles = zeros(size(obstacles));
for i = 1:size(obstacles, 4)
  extraArgs.obstacles(:,:,:,i) = ...
    migrateGrid(old_g, obstacles(:,:,:,i), schemeData.grid);
end

% Computation should stop once it contains the initial state
extraArgs.stopSetIntersect = ...
  migrateGrid(old_g, vehicle.data.targetsm, schemeData.grid);

% Compute the FRS
extraArgs.visualize = true;
[vehicle.data.FRS1, vehicle.data.FRS1_tau] = ...
  HJIPDE_solve(extraArgs.targets, tauFRS, schemeData, 'none', extraArgs);

% Extract ETA
vehicle.data.ETA = vehicle.data.FRS1_tau(end);
end