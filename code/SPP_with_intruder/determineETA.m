function vehicle = determineETA(vehicle, tauFRS, schemeData, obstacles)
% ETA = determineETA(vehicle, tauFRS, schemeData, obstacles)
%     Computes the ETA of a vehicle

schemeData.uMode = 'max';
schemeData.dMode = 'min';
schemeData.tMode = 'forward';

% Set obstacles
extraArgs.obstacles = obstacles;

% Min with target
min_dx = min(schemeData.grid.dx);
extraArgs.targets = shapeSphere(schemeData.grid, vehicle.x, min_dx);

% Computation should stop once it contains the initial state
extraArgs.stopSetIntersect = vehicle.data.targetsm;

[vehicle.data.FRS1, vehicle.data.FRS1_tau] = ...
  HJIPDE_solve(extraArgs.targets, tauFRS, schemeData, 'zero', extraArgs);

vehicle.data.ETA = vehicle.data.FRS1_tau(end);
end