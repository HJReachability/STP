function computeBRSFaSTrack(obj, SPPP_obj, BRS1_tau, g, staticObs, obstacles, SPPP_folder, ...
  veh, low_memory)
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

% Planner dynamics
pMax = SPPP_obj.pMax;
schemeData.dynSys = Plane2D(obj.x, pMax(1), pMax(2));

%% Visualization
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.plotData.plotDims = [1, 1];
extraArgs.plotData.projpt = [];
if low_memory
  extraArgs.low_memory = true;
  extraArgs.flip_output = true;
  
  targetsm = single(obj.targetsm);
else
  targetsm = obj.targetsm;
end

if ispc
  folder = sprintf('%s\\%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir %s', folder));
else
  folder = sprintf('%s/%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir -p %s', folder));
end

extraArgs.fig_filename = sprintf('%s/', folder);

%% Set obstacles
%if numDims(staticObs) == 2
%  staticObs = repmat(staticObs, [1 1 size(obstacles.data, 3) size(obstacles.data, 4)]);
%end

% Obstacles for overlapping time indices
small = 1e-4;
obsTau_inds = obstacles.tau < max(BRS1_tau)+small & ...
  obstacles.tau > min(BRS1_tau)-small;
extraArgs.obstacles = flip(obstacles.data(:,:,obsTau_inds), 3);

% Obstacles for elements of BRS1_tau that are smaller than in obstacles.tau
smaller_inds = BRS1_tau < min(obstacles.tau)-small;
if any(smaller_inds)
  staticObsSmaller = repmat(staticObs, [1 1 nnz(smaller_inds)]);
  
  % Concatenation is done "in reverse" to flip the obstacle order for BRS
  extraArgs.obstacles = cat(3, extraArgs.obstacles, staticObsSmaller);
end

% No need to consider elements of BRS1_tau that are larger than in obstacles.tau

% Just in case obstacles is empty
if isempty(extraArgs.obstacles)
  extraArgs = rmfield(extraArgs, 'obstacles');
end

%% Extra solver parameters
% Min with target
extraArgs.targets = obj.targetsm;

% Computation should stop once it contains the initial state
extraArgs.stopInit = obj.x;

[obj.BRS1, tau] = HJIPDE_solve(targetsm, BRS1_tau, schemeData, 'none', ...
  extraArgs);

obj.BRS1_tau = BRS1_tau(end-length(tau)+1:end);

if ~low_memory
  obj.BRS1 = flip(obj.BRS1, 3);
end
end