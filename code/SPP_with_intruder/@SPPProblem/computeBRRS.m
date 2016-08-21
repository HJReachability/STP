function computeBRRS(obj, restart)
% compute_BR_RS(obj, restart)
%     Computes the before-replanning reachable sets for the SPP problem
%
% Inputs:
%     obj - SPPP object
%     restart - set to true to restart and overwrite computation

if nargin < 2
  restart = false;
end

%% Load files
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

if exist(obj.rawObs_filename, 'file')
  fprintf('Loading ''raw'' obstacles...\n')
  load(obj.rawObs_filename)
else
  error('rawObs file not found!')
end

%% Grid and time
schemeData.grid = obj.g;
BRS1_tau = obj.tMin:obj.dt:obj.tTarget;

%% Migrate raw augmented obstacles
rawObsBRS.data = zeros([schemeData.grid.N' length(CARS.tau)]);
for i = 1:length(CARS.tau)
  rawObsBRS.data(:,:,:,i) = ...
    migrateGrid(rawObs.g, rawObs.cylObsBRS(:,:,:,i), schemeData.grid);
end
rawObsBRS.tauIAT = CARS.tau;

%% Problem parameters
if restart
  fprintf('Initializing vehicles and restarting BR RS computation...\n')
  Q = initRTT(obj, RTTRS);
  
  % File name to save RS data
  obj.BR_RS_filename = sprintf('%s_chkpt_%f.mat', mfilename, now);  
else
  if exist(obj.BR_RS_filename, 'file')
    fprintf('Loading checkpoint...\n')
    load(obj.BR_RS_filename)
    Q = {Q1; Q2; Q3; Q4};
  else
    error('BR_RS file (checkpoint) not found!')
  end
end

%% Start the computation of reachable sets
for veh=1:length(Q)
  schemeData.dynSys = Q{veh};
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    fprintf('Gathering obstacles for vehicle %d...\n', veh)
    obstacles = ...
      gatherObstacles(Q(1:veh-1), schemeData, BRS1_tau, 'cylObsBRS');
    
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, BRS1_tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(obj.BR_RS_filename, 'Q1', 'Q2', 'Q3', 'Q4', '-v7.3')
  end
  
  %% Compute the nominal trajectories based on BRS1
  if ~isfield(Q{veh}.data, 'nomTraj')
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh} = computeNomTraj(Q{veh}, schemeData);
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  if ~isfield(Q{veh}.data, 'cylObsBRS')
    fprintf('Augmenting obstacles for vehicle %d\n', veh)
    Q{veh} = augmentObstacles(Q{veh}, schemeData, rawObsBRS);
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'BRS1', 'cylObsBRS'});
[Q1, Q2, Q3, Q4] = Q{:};
obj.BR_sim_filename = sprintf('%s_%f.mat', mfilename, now);
save(obj.BR_sim_filename, 'Q1', 'Q2', 'Q3', 'Q4', '-v7.3')
end