function SPPwIntruder_RS(RTTRS_filename, restart, chkpt_filename, ...
  initStates, targetCenters)
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

%% Grid
grid_min = [-1; -1; -3*pi/2]; % Lower corner of computation domain
grid_max = [1; 1; pi/2];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

%% Time parameters
% For BRS
t0 = -5;
tf = 0;
dt = 0.01;
BRS1_tau = t0:dt:tf;

% For Intruder
tIAT = 0.1;
tauIAT = 0:dt:tIAT;

%% Base obstacle generation method
baseObs_method = 'RTT';
if strcmp(baseObs_method, 'RTT')
  fprintf('Using %s method to generate base obstacles\n', baseObs_method)
  load(RTTRS_filename)
  baseObs_params.RTTRS = ...
    migrateGrid(RTTRS.g, -RTTRS.data(:,:,:,end), schemeData.grid);
  
elseif strcmp(baseObs_method, 'CC')
  %% Reset radius for base obstacle computation
  baseObs_params.resetR = [0.03, 0.03, 0.1]';
  
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
  
  %% Gather induced obstacles of higher-priority vehicles
  % Assume there's no static obstacle
  fprintf('Gathering obstacles for vehicle %d...\n', veh)
  obstacles = ...
    gatherObstacles(Q(1:veh-1), schemeData, BRS1_tau, 'augFlatObsBRS');
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, BRS1_tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute the base obstacles for based on BRS1
  if ~isfield(Q{veh}.data, 'baseObs')
    fprintf('Computing base obstacles for vehicle %d\n', veh)
    if veh < numVeh
      trajOnly = false;
    else
      trajOnly = true;
    end
    
    Q{veh} = computeBaseObs( ...
      Q{veh}, schemeData, baseObs_method, baseObs_params, trajOnly);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Augment base obstacles with t-IAT FRS
  if ~isfield(Q{veh}.data, 'augObsFRS')
    if veh < numVeh
      fprintf('Augmenting base obstacles with FRS for vehicle %d\n', veh)
      Q{veh} = augmentBaseObsFRS(Q{veh}, schemeData, tauIAT);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Flatten augmented obstacles to 3D, add capture radius, and unflatten to 3D
  if ~isfield(Q{veh}.data, 'cylObs3D')
    if veh < numVeh
      fprintf('Flattening obstacles for vehicle %d\n', veh)
      Q{veh} = flatAugObs(Q{veh}, schemeData, Rc, 'augObsFRS');
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  if ~isfield(Q{veh}.data, 'augFlatObsBRS')
    if veh < numVeh
      fprintf('Augmenting flattening obstacles using BRS for vehicle %d\n', veh)
      Q{veh} = augmentFlatObsBRS(Q{veh}, schemeData, tauIAT);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'BRS1', 'baseObs', 'augObsFRS', 'augFlatObsBRS'});
[Q1, Q2, Q3, Q4] = Q{:};
save(mfilename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
end