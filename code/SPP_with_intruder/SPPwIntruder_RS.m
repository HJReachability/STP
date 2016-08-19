function SPPwIntruder_RS(RTTRS_filename, CARS_filename, ...
  Obs_filename, restart, chkpt_filename, initStates, targetCenters)
% This function initializes the simulation for solving the SPP problem in
% the presence of intruder.

if nargin < 4
  restart = false;
end

if nargin < 5
  filename = sprintf('%s_chkpt_%f.mat', mfilename, now);
else
  filename = chkpt_filename ;
end

if nargin < 6
  initStates = ...
    {[-0.6; 0.2; 0]; [ 0.6; 0.2; -pi]; [-0.5; 0.9; -pi/4]; [ 0.5; 0.9; -3*pi/4]};
end

if nargin < 7
  targetCenters = ...
    {[0.7; 0.7; 0]; [-0.7; 0.7; 0]; [0.7; -0.3; 0]; [-0.7; -0.3; 0]};
end

%% Grids
% Main Grid
schemeData.grid = ...
  createGrid([-1; -1; -3*pi/2], [1; 1; pi/2], [101; 101; 101], 3);

%% Time parameters
% For BRS
t0 = -5;
tf = 0;
dt = 0.01;
BRS1_tau = t0:dt:tf;

%% Load robust tracking reachable set (needed for vehicle parameters)
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)

fprintf('Loading CARS...\n')
load(CARS_filename)
tauIAT = CARS.tau;

%% Raw augmented obstacles
fprintf('Loading ''raw'' obstacles...\n')
load(Obs_filename)
rawObsBRS.data = zeros([schemeData.grid.N' length(tauIAT)]);
for i = 1:length(tauIAT)
  rawObsBRS.data(:,:,:,i) = ...
    migrateGrid(rawObs.g, rawObs.cylObsBRS(:,:,:,i), schemeData.grid);
end
rawObsBRS.tauIAT = tauIAT;

%% Problem parameters
targetR = 0.1; % Target radius
if restart
  fprintf('Initializing vehicles...\n')
  Q = initRTT(initStates, targetCenters, targetR, RTTRS, schemeData);
else
  fprintf('Loading checkpoint...\n')
  load(filename)
  Q = {Q1; Q2; Q3; Q4};
end

numVeh = length(Q);

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
   
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    fprintf('Gathering obstacles for vehicle %d...\n', veh)
    obstacles = ...
      gatherObstacles(Q(1:veh-1), schemeData, BRS1_tau, 'cylObsBRS');
  
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, BRS1_tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute the nominal trajectories based on BRS1
  if ~isfield(Q{veh}.data, 'nomTraj')
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh} = computeNomTraj(Q{veh}, schemeData);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  if ~isfield(Q{veh}.data, 'cylObsBRS')
    fprintf('Augmenting obstacles for vehicle %d\n', veh)
    Q{veh} = augmentObstacles(Q{veh}, schemeData, rawObsBRS);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'BRS1', 'cylObsBRS'});
[Q1, Q2, Q3, Q4] = Q{:};
save(sprintf('%s_%f.mat', mfilename, now), 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
end