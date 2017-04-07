function computeNIRS(obj, restart, low_memory, CPP_RTTRS_file, separatedNIRS)
% computeNIRS(obj, restart)
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

if nargin < 4
    CPP_RTTRS_file = false;
end

if nargin < 5
    separatedNIRS = false;
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
  if (CPP_RTTRS_file == 1)
    RTTRS = loadRTTRS(obj.RTTRS_filename);
  else
    load(obj.RTTRS_filename);
  end
else
  error('RTTRS file not found!')
end

% Checkpoint
if separatedNIRS
    first_NI_RS_chkpt_filename = sprintf('%s0.mat', obj.NI_RS_chkpt_filename);
else
    first_NI_RS_chkpt_filename = obj.NI_RS_chkpt_filename;
end
if restart || ~exist(first_NI_RS_chkpt_filename, 'file')
    fprintf('Initializing vehicles and restarting BR RS computation...\n')
    Q = initRTT(obj, RTTRS);
      % File name to save RS data
      obj.NI_RS_chkpt_filename = sprintf('%s/%s_chkpt.mat', obj.folder, mfilename);
      vehStart = 1;

      SPPP = obj;
      save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')  
else
    if CPP_RTTRS_file
        if separatedNIRS
            Q = initRTT(obj, RTTRS);
            numVeh = length(obj.tTarget);
            for veh=1:numVeh
                Plane_filename = sprintf('%s%d.mat', obj.NI_RS_chkpt_filename, veh-1);
                if exist(Plane_filename, 'file')
                    % last file may not be completed file.
                    vehStart = veh;
%                    break;
                end
            end
            for vehicle=1:vehStart
                Plane_filename = sprintf('%s%d.mat', obj.NI_RS_chkpt_filename, vehicle-1);
                if exist(Plane_filename, 'file')
                   fprintf('Loading %dth vehicle...\n', vehicle-1)
                   load(Plane_filename);
                   Q{vehicle} = cpp2matSPPPlane(Qthis, obj.gN);
                else
 %                   break;
                end
            end
            if exist('obstacles', 'var')
                obstacles.tau = double(obstacles.tau);
                for t=1:length(obstacles.data)
                    tmpObs(:,:,:,t) = double(reshape(obstacles.data{t},obj.gN));
                end
                obstacles.data = tmpObs;
            end
    
        else
           Q = loadNIRS(obj.NI_RS_filename, obj.gN);
        end
    else
      fprintf('Loading NI RS checkpoint...\n')
      load(obj.NI_RS_chkpt_filename)
      vehStart = veh;
    end
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
    thisTau = obj.tTarget(veh)-obj.max_BRS_time:obj.dt:obj.tTarget(veh);
  end
  
  %% Update obstacle
  if veh == 1
    obstacles.tau = thisTau;
    obstacles.data = repmat(obj.augStaticObs, [1 1 obj.g.N(3) length(thisTau)]);
  else
    if ~isempty(Q{veh-1}.obsForRTT)
      fprintf('Updating obstacles for vehicle %d...\n', veh)
      
      old_tau_inds = obstacles.tau > obj.tTarget(veh) + small;
      obstacles.tau(old_tau_inds) = [];
      obstacles.data(:,:,:,old_tau_inds) = [];
      
      obstacles = updateObstacles(obstacles, Q{veh-1}.obsForRTT_tau, ...
        Q{veh-1}.obsForRTT, obj.augStaticObs);
      
      fprintf('Trimming obstacle data and saving checkpoint...\n')
      Q{veh-1}.trimData({'obsForRTT'});
      save(obj.NI_RS_chkpt_filename, 'Q', 'obstacles', 'veh', '-v7.3');
      
      close all
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
    
    Q{veh}.computeObsForRTT(obj, RTTRS);
    if veh == length(Q)
      Q{veh}.trimData({'obsForRTT'});
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