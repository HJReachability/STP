function vehicle = computeBRS1(vehicle, BRS1_tau, schemeData, obstacles)
% vehicle = computeBRS1(vehicle, tau, schemeData, obstacles)
%     Computes the first BRS for a vehicle, and updates its data with
%     BRS1_tau and BRS1 fields. This BRS is used for optimally getting to the
%     target despite the worst-case disturbances and moving obstacles induced by
%     other vehicles
%
% Inputs
%     vehicle:
%         vehicle for which the BRS is computed
%     tau:
%         the times for which the BRS is computed; unused times will be cut off
%     schemeData:
%         parameters for HJI PDE solver, should contain the following fields:
%         .dynSys, .grid
%     obstacles:
%         union of induced obstacles from higher-priority vehicles
%
% Output
%     vehicle:
%         updated vehicle object

% using same tau as FRS is causing BRS to not include target

% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% Set extraArgs
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = vehicle.x(3);

% Set obstacles
extraArgs.obstacles = obstacles;

% Min with target
extraArgs.targets = vehicle.data.targetsm;

% Computation should stop once it contains the initial state
extraArgs.stopInit = vehicle.x;

[vehicle.data.BRS1, tau] = ...
  HJIPDE_solve(vehicle.data.targetsm, BRS1_tau, schemeData, 'none', extraArgs);

% Reverse the order of time elements
vehicle.data.BRS1_tau = BRS1_tau(end-length(tau)+1:end);
vehicle.data.BRS1 = flip(vehicle.data.BRS1, 4);
end