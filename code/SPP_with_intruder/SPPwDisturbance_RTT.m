function SPPwDisturbance_RTT(RTTRS_filename, restart, chkpt_filename, ...
  initStates, targetCenters)
% SPPwDisturbance_RTT(RTTRS_filename, restart, chkpt_filename, ...
%   initStates, targetCenters)
%     Computes all the reachable sets required for running the SPP with
%     disturbances problem using the robust trajectory tracking method

%% Inputs
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
    {[-0.5; 0; 0]; [ 0.5; 0; -pi]; [-0.6; 0.6; -pi/4]; [ 0.6; 0.6; -3*pi/4]};
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
t0 = -5;
tf = 0;
dt = 0.01;
BRS1_tau = t0:dt:tf;

%% Base obstacle generation method
baseObs_method = 'RTT';
fprintf('Using %s method to generate base obstacles\n', baseObs_method)
load(RTTRS_filename)
baseObs_params.RTTRS = ...
  migrateGrid(RTTRS.g, -RTTRS.data(:,:,:,end), schemeData.grid);

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
    obstacles = gatherObstacles(Q(1:veh-1), schemeData, BRS1_tau, 'cylObs3D');    
    
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
  
  %% Flatten obstacles to 3D, add capture radius, and unflatten to 3D
  if ~isfield(Q{veh}.data, 'cylObs3D')
    if veh < numVeh
      fprintf('Flattening obstacles for vehicle %d\n', veh)
      Q{veh} = flatAugObs(Q{veh}, schemeData, Rc, 'baseObs');
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'BRS1', 'baseObs'});
[Q1, Q2, Q3, Q4] = Q{:};
save(mfilename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
end