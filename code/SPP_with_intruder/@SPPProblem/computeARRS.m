function computeARRS(obj, restart)
% SPPwIntruder_replan_RS(restart, chkpt_filename)
%     Computes BRSs for replanning after an intruder has passed
%     CAUTION: This function assumes that the RTT method is used!

if nargin < 2
  restart = false;
end

if restart || ~exist(obj.AR_RS_filename, 'file')
  fprintf('Loading BR sim file and restarting AR RS computation...\n')
  load(obj.BR_sim_filename)
  
  % File name to save RS data
  obj.AR_RS_filename = sprintf('%s_RS_%f.mat', mfilename, now);  
else
  fprintf('Loading AR RS checkpoint...\n')
  load(obj.AR_RS_filename)
end
Q = {Q1; Q2; Q3; Q4};

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

%% Grid and time
schemeData.grid = obj.g;
tFRS_max = 2;
tauFRS = obj.tReplan:obj.dt:tFRS_max;

%% Flattening and adding capture radius to RTTRS
RTTRS_temp = migrateGrid(RTTRS.g, -RTTRS.data, schemeData.grid);
[augRTTRS2D.g, RTTRS2D] = proj(schemeData.grid, RTTRS_temp, [0 0 1]);
augRTTRS2D.data = addCRadius(augRTTRS2D.g, RTTRS2D, CARS.Rc);

%% Start the computation of reachable sets
for veh=1:length(Q)
  if Q{veh}.data.replan
    schemeData.dynSys = Q{veh};
    
    %% Gather induced obstacles for FRS computation
    % Assume there's no static obstacle
    fprintf('Gathering obstacles for vehicle %d for FRS computation...\n', veh)
    obstacles = ...
      gatherObstacles(Q(1:veh-1), schemeData, tauFRS, 'cylObs', 'forward');
    
    %% Compute FRS to determine the ETA
    if ~isfield(Q{veh}.data, 'ETA')
      fprintf('Determining ETA for vehicle %d\n', veh)
      Q{veh} = determineETA(Q{veh}, tauFRS, schemeData, obstacles);
      
      [Q1, Q2, Q3, Q4] = Q{:};
      save(obj.AR_RS_filename, 'Q1', 'Q2', 'Q3', 'Q4', '-v7.3')
    end
    
    %% Compute the BRS (BRS1) of the vehicle with the above obstacles
    if ~isfield(Q{veh}.data, 'BRS1')
      tauBRS = Q{veh}.data.FRS1_tau;
      fprintf('Gathering obstacles for vehicle %d for BRS computation...\n',veh)
      obstacles = ...
        gatherObstacles(Q(1:veh-1), schemeData, tauBRS, 'cylObs', 'backward');
      
      fprintf('Computing BRS1 for vehicle %d\n', veh)
      Q{veh} = computeBRS1(Q{veh}, tauBRS, schemeData, obstacles);
      
      [Q1, Q2, Q3, Q4] = Q{:};
      save(obj.AR_RS_filename, 'Q1', 'Q2', 'Q3', 'Q4', '-v7.3')
    end
    
    %% Compute the nominal trajectories based on BRS1
    if ~isfield(Q{veh}.data, 'nomTraj')
      fprintf('Computing nominal trajectory for vehicle %d\n', veh)
      Q{veh} = computeNomTraj(Q{veh}, schemeData);
    end
  end
  
  %% Compute induced obstacles
  if ~isfield(Q{veh}.data, 'cylObs')
    fprintf('Computing induced obstacles for vehicle %d\n', veh)
    Q{veh} = computeCylObs(Q{veh}, schemeData, augRTTRS2D);
  end
end

%% Trim vehicles for a smaller file
Q = trimDataForSim(Q, {'FRS1', 'BRS1', 'cylObs'});
[Q1, Q2, Q3, Q4] = Q{:};
obj.AR_RS_filename_small = sprintf('%s_sim_%f.mat', mfilename, now);
save(obj.AR_RS_filename_small, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
end