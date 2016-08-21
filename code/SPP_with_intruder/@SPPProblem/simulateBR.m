function simulateBR(obj, intrIS, intrCtrl, tIntr, save_png, save_fig)
%% Default inputs
if nargin < 2
  intrIS = [-0.75; 0.1; 0*pi/180];
end

if nargin < 3
  intrCtrl = [0.75; 0];
end

if nargin < 4
  tIntr = -2.5;
end

if nargin < 5
  save_png = true;
end

if nargin < 6
  save_fig = false;
end

small = 1e-4;

%% Load files
% Load robust tracking reachable set
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

% Load safety reachable set
if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

% Load path planning reachable set
if exist(obj.CARS_filename, 'file')
  fprintf('Loading before-replanning RS...\n')
  load(obj.BR_RS_filename)
else
  error('Before-replanning RS file not found!')
end

% Load raw obstacles file
if exist(obj.rawObs_filename, 'file')
  fprintf('Loading ''raw'' obstacles...\n')
  load(Obs_filename)
else
  error('Raw obstacles file not found!')
end

%% Post process loaded data
% Compute gradients used for optimal control
fprintf('Computing gradients...')
RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data);
CARS.Deriv = computeGradients(CARS.g, CARS.data);

% Determine time of simulation
Q = {Q1;Q2;Q3;Q4};
tStart = inf;
tEnd = -inf;
for veh = 1:length(Q)
  tStart = min(tStart, min(Q{veh}.data.nomTraj_tau));
  tEnd = max(tEnd, max(Q{veh}.data.nomTraj_tau));
end
tau = tStart:obj.dt:tEnd;

% Add cylindrical obstacles for visualization
if save_png || save_fig
  rawCylObs.data = zeros([schemeData.grid.N' length(CARS.tau)]);
  for i = 1:length(CARS.tau)
    rawCylObs.data(:,:,:,i) = ...
      migrateGrid(rawObs.g, rawObs.cylObs3D(:,:,:,i), schemeData.grid);
  end
  rawCylObs.tauIAT = CARS.tau;
  
  for veh = 1:length(Q)
    fprintf('Adding cylindrical obstacles vehicle %d for visualization...\n', veh)
    Q{veh} = addCylObs(Q{veh}, schemeData, rawCylObs);
  end
  
  % For saving graphics
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
end

%% Initialize figure
if save_png || save_fig
  f = figure;
  schemeData.grid = obj.g;
  colors = lines(length(Q));
  plotTargetSets(Q, schemeData, colors)
  
  hc = cell(length(Q), 1); % Capture radius
  ho = cell(length(Q), 1); % Obstacle
  hn = cell(length(Q), 1); % Nominal trajectory
  
  intruder_color = 'k';
end

%% Initialize intruder
Qintr = Plane(intrIS, CARS.dynSys.wMaxB, CARS.dynSys.vRangeB, ...
  CARS.dynSys.dMaxB);

tReplan = inf;

safety_vals = 1e3*ones(length(Q), length(tau));
safety_threshold = 0.5;
intruder_arrived = false;

% Keep track of which vehicles need to replan
last_replan_veh = length(Q)+1;
for veh = 1:length(Q)
  Q{veh}.data.replan = false;
end

%% Simulate
tInds = cell(length(Q),1);
safety_rel_x = cell(length(Q),1);
for i = 1:length(tau)
  fprintf('t = %f\n', tau(i))
  
  % Check if nominal trajectory has this t
  for veh = 1:length(Q)
    tInds{veh} = find(Q{veh}.data.nomTraj_tau > tau(i) - small & ...
      Q{veh}.data.nomTraj_tau < tau(i) + small, 1);
  end
  
  %% Intruder
  if tau(i) >= tIntr && tau(i) <= tReplan
    intrDstb = Qintr.uniformDstb();
    Qintr.updateState(intrCtrl, dt, Qintr.x, intrDstb);
    Qintr.plotPosition(intruder_color);
    
    % Check safety
    for veh = 1:length(Q)
      % Compute safety value
      if ~isempty(tInds{veh})
        safety_rel_x{veh} = Qintr.x - Q{veh}.x;
        safety_rel_x{veh}(1:2) = rotate2D(safety_rel_x{veh}(1:2), -Q{veh}.x(3));
        safety_rel_x{veh}(3) = wrapTo2Pi(safety_rel_x{veh}(3));
        safety_vals(veh, i) = eval_u(CARS.g, CARS.data, safety_rel_x{veh});
      end
    end
    
    % Mark time at which intruder shows up
    if ~intruder_arrived && any(safety_vals(:, i) < safety_threshold)
      tReplan = tau(i) + max(CARS.tau);
      intruder_arrived = true;
    end
  end
  
  if tau(i) <= tReplan
    %% Control and disturbance for SPP Vehicles
    for veh = 1:length(Q)
      if ~isempty(tInds{veh})
        if safety_vals(veh, i) < safety_threshold
          fprintf('Vehicle %d is performing avoidance.\n', veh)
          
          % Safety controller
          deriv = eval_u(CARS.g, CARS.Deriv, safety_rel_x{veh});
          u = CARS.dynSys.optCtrl([], safety_rel_x{veh}, deriv, 'max');
          
          last_replan_veh = min(last_replan_veh, veh);
        else
          liveness_rel_x = Q{veh}.data.nomTraj(:,tInds{veh}) - Q{veh}.x;
          liveness_rel_x(1:2) = rotate2D(liveness_rel_x(1:2), -Q{veh}.x(3));
          deriv = eval_u(RTTRS.g, RTTRS.Deriv, liveness_rel_x);
          
          u = RTTRS.dynSys.optCtrl([], liveness_rel_x, deriv, 'max');
        end
        % Random disturbance
        d = Q{veh}.uniformDstb();
        Q{veh}.updateState(u, dt, Q{veh}.x, d);
      end
    end
    
    %% Visualize
    if save_png || save_fig
      [hc, ho, hn] = plotVehicles(Q, tInds, schemeData, hc, ho, hn, ...
        colors, CARS.Rc);
      
      xlim([-1.2 1.2])
      ylim([-1.2 1.2])
      
      title(sprintf('t = %f', tau(i)))
      drawnow;
    end
    
    % Save plots
    if save_png
      export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
    end
    
    if save_fig
      savefig(f, sprintf('%s/%d', folder, i), 'compact')
    end
  else
    fprintf('Saving data for replanning.\n')
    for veh = last_replan_veh:length(Q)
      Q{veh}.data.replan = true;
    end
    
    obj.tIntr = tIntr;
    obj.tReplan = tau(i);
    obj.BR_sim_filename = sprintf('Replan_RS_%f.mat', now);
    saveReplanData(Q, Qintr, obj.BR_sim_filename);
    return
  end
end
end