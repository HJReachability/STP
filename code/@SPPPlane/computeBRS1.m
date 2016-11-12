function computeBRS1(obj, BRS1_tau, g, obstacles, SPPP_folder, veh)
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

%% Set schemeData
schemeData.grid = g;
schemeData.uMode = 'min';

% Modify control bounds
nom_vrange = obj.vrange + obj.vReserved;
nom_wMax = obj.wMax + obj.wReserved;
schemeData.dynSys = Plane(obj.x, nom_wMax, nom_vrange);

% Set extraArgs
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = obj.x(3);

if ispc
  folder = sprintf('%s\\%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir %s', folder));
else
  folder = sprintf('%s/%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir -p %s', folder));
end

extraArgs.fig_filename = sprintf('%s/', folder);

% Set obstacles
extraArgs.obstacles = obstacles;

% Min with target
extraArgs.targets = obj.targetsm;

% Computation should stop once it contains the initial state
extraArgs.stopInit = obj.x;

[BRS1, tau] = HJIPDE_solve(obj.targetsm, BRS1_tau, schemeData, 'none', ...
  extraArgs);

% Reverse the order of time elements
obj.BRS1_tau = BRS1_tau(end-length(tau)+1:end);
obj.BRS1 = flip(BRS1, 4);
end