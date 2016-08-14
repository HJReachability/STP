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

%% Load data for replanning
load(Replan_filename)

%% Load RTT reachable set
baseObs_method = 'RTT';
fprintf('Using %s method to generate base obstacles\n', baseObs_method)
load(RTTRS_filename)
baseObs_params.RTTRS = ...
  migrateGrid(RTTRS.g, -RTTRS.data(:,:,:,end), schemeData.grid);

%% Time vector
dt = 0.01;

% Time vector for FRS
tFRS_max = 2;
tauFRS = 0:dt:tFRS_max;

if ~restart
  load(filename)
  Q = {Q1; Q2; Q3; Q4};
end
numVeh = length(Q);

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
  
  %% Gather induced obstacles for FRS computation
  % Assume there's no static obstacle
  if veh > 1
    fprintf('Gathering obstacles for vehicle %d for FRS computation...\n', veh)
    obstacles = ...
      gatherObstacles(Q(1:veh-1), schemeData, tau, 'cylObs3D', 'forward');
    
    %% Compute FRS to determine the ETA
    if ~isfield(Q{veh}.data, 'ETA')
      fprintf('Determining ETA for vehicle %d\n', veh)
      vehicle = determineETA(vehicle, tauFRS, schemeData, obstacles, tNow);
      
      [Q1, Q2, Q3, Q4] = Q{:};
      save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
    end
  end
  
  %% Gather induced obstacles for BRS computation
  fprintf('Gathering obstacles for vehicle %d for BRS computation...\n', veh)
  obstacles = ...
    gatherObstacles(Q(1:veh-1), schemeData, tau, 'cylObs3D', 'backward');
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    tauBRS = vehicle.data.ETA-tFRS_max:dt:vehicle.data.ETA;
    Q{veh} = computeBRS1(Q{veh}, tauBRS, schemeData, obstacles);
    
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

end