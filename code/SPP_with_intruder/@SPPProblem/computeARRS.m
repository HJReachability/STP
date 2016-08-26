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

%% Time
tFRS_max = 2;
tauFRS = obj.tReplan:obj.dt:tFRS_max;

%% Start the computation of reachable sets
for veh = 1:length(Q)
  if Q{veh}.replan
    %% Compute FRS to determine the ETA
    if isempty(Q{veh}.FRS1)
      % Assume there's no static obstacle
      fprintf('Gathering obstacles for vehicle %d for FRS computation...\n', veh)
      obstacles = ...
        gatherObstacles(Q(1:veh-1), obj.g, tauFRS, 'obsForRTT', 'forward');
      fprintf('Computing FRS1 for vehicle %d\n', veh)
      Q{veh}.computeFRS1(tauFRS, obj.g, obstacles);
      
      [Q1, Q2, Q3, Q4] = Q{:};
      save(obj.AR_RS_filename, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
    end
    
    %% Compute the BRS (BRS1) of the vehicle with the above obstacles
    if isempty(Q{veh}.BRS1)
      tauBRS = Q{veh}.FRS1_tau;
      fprintf('Gathering obstacles for vehicle %d for BRS computation...\n',veh)
      obstacles = ...
        gatherObstacles(Q(1:veh-1), obj.g, tauBRS, 'obsForRTT', 'backward');
      
      fprintf('Computing BRS1 for vehicle %d\n', veh)
      Q{veh}.computeBRS1(tauBRS, obj.g, obstacles);
      
      [Q1, Q2, Q3, Q4] = Q{:};
      save(obj.AR_RS_filename, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
    end
    
    %% Compute the nominal trajectories based on BRS1
    % Continued from before replanning!
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh}.computeNomTraj(obj.g);
  end
  
  %% Compute induced obstacles
  if isempty(Q{veh}.obsForRTT)
    fprintf('Computing induced obstacles for vehicle %d\n', veh)
    Q{veh}.computeObsForRTT(obj, RTTRS);
  end
end

%% Trim vehicles for a smaller file
for veh = 1:length(Q)
  Q{veh}.trimData({'FRS1', 'BRS1', 'obsForRTT'});
end
[Q1, Q2, Q3, Q4] = Q{:};
obj.AR_RS_filename_small = sprintf('%s_sim_%f.mat', mfilename, now);
save(obj.AR_RS_filename_small, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
end