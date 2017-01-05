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

%% Check to see if 
if ~restart && exist(obj.NI_RS_filename, 'file')
  fprintf('The NI RS file %s already exists. Skipping NI RS computation.\n', ...
    obj.NI_RS_filename)
  return
end

%% Load files
% RTTRS
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

% Checkpoint
if restart || ~exist(obj.NI_RS_chkpt_filename, 'file')
  fprintf('Initializing vehicles and restarting BR RS computation...\n')
  Q = initRTT(obj, RTTRS);
  
  % File name to save RS data
  obj.NI_RS_chkpt_filename = sprintf('%s/%s_chkpt.mat', obj.folder, mfilename);
  
  SPPP = obj;
  save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')  
else
  fprintf('Loading NI RS checkpoint...\n')
  load(obj.NI_RS_chkpt_filename)
end

if ispc
  data_folder = sprintf('%s\\Plane_data', obj.folder);
else
  data_folder = sprintf('%s/Plane_data', obj.folder);
end
system(sprintf('mkdir %s', data_folder));

%% Start the computation of reachable sets
for veh = 1:length(Q)
  % Potential time stamps for current vehicle
  if length(obj.tTarget) == 1
    thisTau = obj.tau;
  else
    thisTau = obj.tTarget(veh)-500:obj.dt:obj.tTarget(veh);
  end
  
  %% Update obstacle
  if veh == 1
    obstacles.tau = thisTau;
    obstacles.data = repmat(obj.augStaticObs, [1 1 obj.g.N(3) length(thisTau)]);
  else
    if ~isempty(Q{veh-1}.obsForRTT)
      fprintf('Updating obstacles for vehicle %d...\n', veh)
      obstacles = updateObstacles(obstacles, Q{veh-1}.obsForRTT_tau, ...
        Q{veh-1}.obsForRTT);
      Q{veh-1}.trimData({'obsForRTT'});
      save(obj.NI_RS_chkpt_filename, 'Q', 'obstacles', 'veh', '-v7.3');
    end
  end
  
  if isempty(Q{veh}.nomTraj)
    %% Compute the BRS (BRS1) of the vehicle with the above obstacles
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh}.computeBRS1(thisTau, obj.g, obstacles, obj.folder, veh);
    
    %% Compute the nominal trajectories based on BRS1
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh}.computeNomTraj(obj.g, obj.folder, veh);
    
    %% Compute induced obstacles
    fprintf('Computing obstacles for vehicle %d\n', veh)
    if veh < length(Q)
      Q{veh}.computeObsForRTT(obj, RTTRS);
    else
      save(obj.NI_RS_chkpt_filename, 'Q', 'obstacles', 'veh', '-v7.3');
    end
    
    Qthis = Q{veh};
    save(sprintf('%s/Plane%d.mat', data_folder, veh), 'Qthis', '-v7.3')
    Q{veh}.trimData({'BRS1'});
  end
end

obj.NI_RS_filename = sprintf('%s/%s.mat', obj.folder, mfilename);
save(obj.NI_RS_filename, 'Q', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end