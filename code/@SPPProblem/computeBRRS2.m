function computeBRRS2(obj, restart, low_memory)
% compute_BR_RS(obj, restart)
%     Computes the before-replanning reachable sets for the SPP problem
%
% Inputs:
%     obj - SPPP object
%     restart - set to true to restart and overwrite computation

if nargin < 2
  restart = false;
end

if nargin < 3
  low_memory = false;
end

if ~restart && exist(obj.BR_RS_filename, 'file')
  fprintf('The BR RS file %s already exists. Skipping BR RS computation.\n', ...
    obj.BR_RS_filename)
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

% raw FRS BRS data
if exist(obj.FRSBRS_filename, 'file')
  fprintf('Loading FRSBRS file...\n')
  load(obj.FRSBRS_filename)
else
  error('FRSBRS file not found!')
end

% Buffer region data
if exist(obj.bufferRegion_filename, 'file')
  fprintf('Loading buffer region file...\n')
  load(obj.bufferRegion_filename)
else
  error('buffer region file not found!')
end

if restart || ~exist(obj.BR_RS_chkpt_filename, 'file')
  fprintf('Initializing vehicles and restarting BR RS computation...\n')
  Q = initRTT(obj, RTTRS);
  
  % File name to save RS data
  obj.BR_RS_chkpt_filename = sprintf('%s/%s_chkpt.mat', obj.folder, mfilename);
  vehStart = 1;
else
  fprintf('Loading BR RS checkpoint...\n')
  load(obj.BR_RS_chkpt_filename)
  vehStart = veh;
end

if ispc
  data_folder = sprintf('%s\\Plane_data', obj.folder);
else
  data_folder = sprintf('%s/Plane_data', obj.folder);
end
system(sprintf('mkdir %s', data_folder));

small = 1e-3;

%% Start the computation of reachable sets
for veh = vehStart:length(Q)
  % Potential time stamps for current vehicle
  if length(obj.tTarget) == 1
    thisTau = obj.tau;
  else
    thisTau = obj.tTarget(veh)-500:obj.dt:obj.tTarget(veh);
    old_tau_inds = obstacles.tau > obj.tTarget(veh) + small;
    obstacles.tau(old_tau_inds) = [];
    obstacles.data(:,:,:,old_tau_inds) = [];
  end
  
  %% Update obstacle
  if veh == 1
    obstacles.tau = thisTau;
    obstacles.data = repmat(single(obj.augStaticObs), [1 1 1 length(thisTau)]);
  else
    if ~isempty(Q{veh-1}.obsForIntr)
      fprintf('Updating obstacles for vehicle %d...\n', veh)
      obstacles = updateObstacles(obstacles, Q{veh-1}.obsForIntr_tau, ...
        Q{veh-1}.obsForIntr, obj.augStaticObs);
      
      Q{veh-1}.trimData({'obsForIntr'});
      save(obj.BR_RS_chkpt_filename, 'Q', 'obstacles', 'veh', '-v7.3');
    end
  end
  
  if isempty(Q{veh}.nomTraj)
    %% Compute the BRS (BRS1) of the vehicle with the above obstacles
    fprintf('Computing BRS1 for vehicle %d\n', veh)

    Q{veh}.computeBRS1(thisTau, obj.g, obj.augStaticObs, obstacles, ...
      obj.folder, veh, low_memory);
    
    %% Compute the nominal trajectories based on BRS1
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    Q{veh}.computeNomTraj(obj.g, obj.folder, veh);
    
    %% Compute induced obstacles
    fprintf('Computing obstacles for vehicle %d\n', veh)
    Q{veh}.computeObsForIntr2(obj, bufferRegion, FRSBRS, veh);
    if veh == length(Q)
      Q{veh}.trimData({'obsForIntr'});
      save(obj.BR_RS_chkpt_filename, 'Q', 'obstacles', 'veh', '-v7.3');
    end
    
    Qthis = Q{veh};
    save(sprintf('%s/Plane%d.mat', data_folder, veh), 'Qthis', '-v7.3')
    Q{veh}.trimData({'BRS1'});
  end
end

obj.BR_RS_filename = sprintf('%s/%s.mat', obj.folder, mfilename);
save(obj.BR_RS_filename, 'Q', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end