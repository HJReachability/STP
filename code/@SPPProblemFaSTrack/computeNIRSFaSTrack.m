function computeNIRSFaSTrack(obj, restart, low_memory, CPP_RTTRS_file, separatedNIRS)
% computeNIRSFaSTrack(obj, restart)
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
    separatedNIRS = false;   %%%%%%%%%%%%%%%%%%
end

%% Augment static obstacles - used to be in loadSetup
if isfield(obj.extraArgs, 'dstb_or_intr') && ...
    strcmp(obj.extraArgs.dstb_or_intr, 'dstb') == 1

    augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.trackingRadius);
          
    % Set boundary of grid to also be a static obstacle, function is negative
    % to help with taking the min (next line)
    if (strcmp(obj.setupName, 'room') == 1)
        obs_bdry = -shapeRectangleByCorners(obj.g2D, obj.g2D.min + [0.1;0.1], ...
       obj.g2D.max - [0.1;0.1]);  
    else
       obs_bdry = -shapeRectangleByCorners(obj.g2D, obj.g2D.min + [5;5], ...
       obj.g2D.max - [5;5]);  
    end 
      
        
    % Take union of grid boundary and the rest of the (augmented) static obstacles 
    obj.augStaticObs = min(augStaticObs, obs_bdry);
    SPPP = obj;
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
    P = initPlanner(obj, RTTRS);
      % File name to save RS data
      obj.NI_RS_chkpt_filename = sprintf('%s/%s_chkpt.mat', obj.folder, mfilename);
      vehStart = 1;

      SPPP = obj;  
      save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')  
else %%%%%%%%%%%%%%%%%%%%%%%%%%%%% all this block
    if CPP_RTTRS_file
        if separatedNIRS
            P = initPlanner(obj, RTTRS);
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
                   P{vehicle} = cpp2matSPPPlane(Pthis, obj.gN);
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
           P = loadNIRS(obj.NI_RS_filename, obj.gN);
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
for veh = vehStart:length(P)
  % Potential time stamps for current vehicle
  if length(obj.tTarget) == 1
    thisTau = obj.tau;
  else
    thisTau = obj.tTarget(veh)-obj.max_BRS_time:obj.dt:obj.tTarget(veh);  
  end
  
  %% Update obstacle
  if veh == 1
    obstacles.tau = thisTau;
    %expand static obstacles into the time dimension
    obstacles.data = repmat(obj.augStaticObs, [1 1 length(thisTau)]);
  else
    if ~isempty(P{veh-1}.obsForRTT)
      fprintf('Updating obstacles for vehicle %d...\n', veh)
      
      %get rid of all obstacles that are present after this vehicle reaches
      %its target
      old_tau_inds = obstacles.tau > obj.tTarget(veh) + small;
      obstacles.tau(old_tau_inds) = [];   
      obstacles.data(:,:,old_tau_inds) = [];
      
      %update all obstacles present in this vehicle's time vector
      obstacles = updateObstaclesFaSTrack(obstacles, P{veh-1}.obsForRTT_tau, ...
        P{veh-1}.obsForRTT, obj.augStaticObs);  
      
      fprintf('Trimming obstacle data and saving checkpoint...\n')
      P{veh-1}.trimDataFaSTrack({'obsForRTT'});
      save(obj.NI_RS_chkpt_filename, 'P', 'obstacles', 'veh', '-v7.3');
      
      close all
    end
  end
  
  if isempty(P{veh}.nomTraj)
    %% Compute the BRS (BRS1) of the vehicle with the above obstacles
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    P{veh}.computeBRSFaSTrack(SPPP, thisTau, obj.g2D, obj.augStaticObs, obstacles, ...
      obj.folder, veh, low_memory);   
    
    %% Compute the nominal trajectories based on BRS1
    fprintf('Computing nominal trajectory for vehicle %d\n', veh)
    P{veh}.computeNomTrajFaSTrack(SPPP, obj.g2D, obj.folder, veh);
    
    %% Compute induced obstacles
    fprintf('Computing obstacles for vehicle %d\n', veh)
    
    P{veh}.computeObsForRTTFaSTrack(obj, RTTRS);
    if veh == length(P)
      P{veh}.trimDataFaSTrack({'obsForRTT'});
      save(obj.NI_RS_chkpt_filename, 'P', 'obstacles', 'veh', '-v7.3');
    end
    
    Pthis = P{veh};
    save(sprintf('%s/Plane%d.mat', data_folder, veh), 'Pthis', '-v7.3')
    P{veh}.trimDataFaSTrack({'BRS1'});
  end
end

obj.P = P;

obj.NI_RS_filename = sprintf('%s/%s.mat', obj.folder, mfilename);
save(obj.NI_RS_filename, 'P', '-v7.3')

SPPP = obj;  
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end