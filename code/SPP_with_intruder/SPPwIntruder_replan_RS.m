function SPPwIntruder_replan_RS(RTTRS_filename, Replan_filename, ...
  restart, chkpt_filename)
% SPPwIntruder_replan_RS(restart, chkpt_filename)
%     Computes BRSs for replanning after an intruder has passed
%     CAUTION: This function assumes that the RTT method is used!

if nargin < 3
  restart = false;
end

if nargin < 4
  filename = sprintf('%s_%f.mat', mfilename, now);
else
  filename = chkpt_filename ;
end

if restart
  fprintf('Loading from replan data file...\n')
  load(Replan_filename)
  Q = {Q1; Q2; Q3; Q4};
else
  fprintf('Loading from checkpoint file...\n')
  load(filename)
  Q = {Q1; Q2; Q3; Q4};
end

%% Load robust tracking reachable set (needed for vehicle parameters)
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)

%% Raw augmented obstacles
fprintf('Loading ''raw'' obstacles...\n')
load(Obs_filename)
rawCylObs.data = zeros([schemeData.grid.N' length(tauIAT)]);
for i = 1:length(tauIAT)
  rawCylObs.data(:,:,:,i) = ...
    migrateGrid(rawObs.g, rawObs.cylObs3D(:,:,:,i), schemeData.grid);
end
rawCylObs.tauIAT = tauIAT;

%% Time vector
dt = 0.01;

% Time vector for FRS
tFRS_max = 2;
tauFRS = tNow:dt:tFRS_max;

numVeh = length(Q);

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
  
  %% Gather induced obstacles for FRS computation
  % Assume there's no static obstacle
  fprintf('Gathering obstacles for vehicle %d for FRS computation...\n', veh)
  obstacles = ...
    gatherObstacles(Q(1:veh-1), schemeData, tauFRS, 'cylObs3D', 'forward');
  
  %% Compute FRS to determine the ETA
  if ~isfield(Q{veh}.data, 'ETA')
    fprintf('Determining ETA for vehicle %d\n', veh)
    Q{veh} = determineETA(Q{veh}, tauFRS, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    tauBRS = Q{veh}.data.FRS1_tau;
    fprintf('Gathering obstacles for vehicle %d for BRS computation...\n', veh)
    obstacles = ...
      gatherObstacles(Q(1:veh-1), schemeData, tauBRS, 'cylObs3D', 'backward');
    
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, tauBRS, schemeData, obstacles);
    
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
  
  %% Compute cylindrical obstacles
  if ~isfield(Q{veh}.data, 'cylObs')
    fprintf('Augmenting obstacles for vehicle %d\n', veh)
    Q{veh} = augmentObstacles(Q{veh}, schemeData, rawCylObs);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end

end