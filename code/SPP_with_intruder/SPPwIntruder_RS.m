function SPPwIntruder_RS(RTTRS_filename, Obs_Filename, restart, ...
  chkpt_filename, initStates, targetCenters)
% This function initializes the simulation for solving the SPP problem in
% the presence of intruder.

if nargin < 2
  restart = false;
end

if nargin < 3
  filename = sprintf('%s_%f.mat', mfilename, now);
else
  filename = chkpt_filename ;
end

if nargin < 4
  initStates = ...
    {[-0.4; 0; 0]; [ 0.4; 0; -pi]; [-0.5; 0.5; -pi/4]; [ 0.5; 0.5; -3*pi/4]};
end

if nargin < 5
  targetCenters = ...
    {[0.7; 0.2; 0]; [-0.7; 0.2; 0]; [0.7; -0.7; 0]; [-0.7; -0.7; 0]};
end

%% Grids
% Main Grid
schemeData.grid = ...
  createGrid([-0.8; -0.8; -3*pi/2], [0.8; 0.8; pi/2], [81; 81; 81], 3);

%% Time parameters
% For BRS
t0 = -5;
tf = 0;
dt = 0.01;
BRS1_tau = t0:dt:tf;

% For Intruder
tIAT = 0.25;
tauIAT = 0:dt:tIAT;

%% Base obstacle generation method
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)
baseObs_params.RTTRS = migrateGrid(RTTRS.g, -RTTRS.data, schemeData.grid);

%% Augment obstacles
fprintf('Loading ''raw'' obstacles...\n')
load(Obs_filename)
rawObsBRS = zeros([schemeData.grid.N' length(tauIAT)]);
for i = 1:length(tauIAT)
  rawObsBRS(:,:,:,i) = ...
    migrateGrid(rawObs.g, rawObs.cylObsBRS(:,:,:,i), schemeData.grid);
end

%% Problem parameters
Rc = 0.1; % Capture radius
targetR = 0.1; % Target radius
if restart
  Q = initRTT(initStates, targetCenters, targetR, RTTRS, schemeData);
else
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
      gatherObstacles(Q(1:veh-1), schemeData, BRS1_tau, 'augFlatObsBRS');
  
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, BRS1_tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute the base obstacles for based on BRS1
  if ~isfield(Q{veh}.data, 'baseObs')
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh} = computeNomTraj(Q{veh}, schemeData);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  if ~isfield(Q{veh}.data, 'cylObsBRS')
    fprintf('Augmenting obstacles for vehicle %d\n', veh)
    Q{veh} = augmentObstacles(Q{veh}, schemeData, tauIAT);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'BRS1', 'baseObs', 'augObsFRS', 'augFlatObsBRS'});
[Q1, Q2, Q3, Q4] = Q{:};
save(mfilename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
end