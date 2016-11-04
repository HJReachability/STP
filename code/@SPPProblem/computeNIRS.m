function computeNIRS(obj, restart)
% computeNIRS(obj, restart)
%     Computes the before-replanning reachable sets for the SPP problem
%
% Inputs:
%     obj - SPPP object
%     restart - set to true to restart and overwrite computation

if nargin < 2
  restart = false;
end

if ~restart && exist(obj.NI_RS_filename_small, 'file')
  fprintf('The NI RS file %s already exists. Skipping NI RS computation.\n', ...
    obj.NI_RS_filename_small)
  return
end

%% Load files
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

if restart || ~exist(obj.NI_RS_filename, 'file')
  fprintf('Initializing vehicles and restarting BR RS computation...\n')
  Q = initRTT(obj, RTTRS);
  
  % File name to save RS data
  obj.NI_RS_filename = sprintf('%s_%f.mat', mfilename, now);  
else
  fprintf('Loading NI RS checkpoint...\n')
  load(obj.NI_RS_filename)
  Q = {Q1; Q2; Q3; Q4};
end

%% Start the computation of reachable sets
for veh = 1:length(Q)
  %% Update obstacle
  fprintf('Updating obstacles for vehicle %d...\n', veh)
  if veh == 1
    obstacles = obj.staticObs;
  else
    obstacles = updateObstacles(obj.tau, obstacles, Q{veh-1}.obsForRTT);
    Q{veh-1}.trimData({'obsForRTT'});
    save(obj.NI_RS_filename, 'Q', '-v7.3');
  end
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if isempty(Q{veh}.BRS1)
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh}.computeBRS1(obj.tau, obj.g, flip(obstacles, 4));
    
    Qthis = Q{veh};
    save(sprintf('Plane%d_dstbRTT.mat', veh), 'Qthis', '-v7.3')
  end
  
  %% Compute the nominal trajectories based on BRS1
  if isempty(Q{veh}.nomTraj)
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh}.computeNomTraj(obj.g);
    
    Qthis = Q{veh};
    save(sprintf('Plane%d_dstbRTT.mat', veh), 'Qthis', '-v7.3')    
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  if isempty(Q{veh}.obsForRTT) && veh < length(Q)
    fprintf('Augmenting obstacles for vehicle %d\n', veh)
    Q{veh}.computeObsForRTT(obj, RTTRS);
    
    Qthis = Q{veh};
    save(sprintf('Plane%d_dstbRTT.mat', veh), 'Qthis', '-v7.3')
    Q{veh}.trimData({'BRS1'});
  end
end

save(obj.NI_RS_filename, 'Q', '-v7.3')
SPPP = obj;
save(obj.this_filename, 'SPPP', '-v7.3')
end