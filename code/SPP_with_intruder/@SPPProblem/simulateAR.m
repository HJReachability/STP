function simulateAR(obj, save_png, save_fig)
% NEED TO USE RTTRS AS OBSTACLES INSTEAD OF RAWOBS

%% Default inputs
if nargin < 2
  save_png = true;
end

if nargin < 3
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
if exist(obj.AR_RS_filename_small, 'file')
  fprintf('Loading after-replanning RS...\n')
  load(obj.AR_RS_filename_small)
else
  error('After-replanning RS file not found!')
end

% Load raw obstacles file
if exist(obj.rawObs_filename, 'file')
  fprintf('Loading ''raw'' obstacles...\n')
  load(obj.rawObs_filename)
else
  error('Raw obstacles file not found!')
end

%% Post process loaded data
% Compute gradients used for optimal control
fprintf('Computing gradients...\n')
RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data);
CARS.Deriv = computeGradients(CARS.g, CARS.data);

% Determine end time of simulation
Q = {Q1;Q2;Q3;Q4};
tEnd = -inf;
for veh = 1:length(Q)
  tEnd = max(tEnd, max(Q{veh}.data.nomTraj_tau));
end
tauAR = obj.tReplan:obj.dt:tEnd;

% Add cylindrical obstacles for visualization
if save_png || save_fig
  rawCylObs.data = zeros([obj.g.N' length(CARS.tau)]);
  for i = 1:length(CARS.tau)
    rawCylObs.data(:,:,:,i) = ...
      migrateGrid(rawObs.g, rawObs.cylObs3D(:,:,:,i), obj.g);
  end
  rawCylObs.tauIAT = CARS.tau;
  
  for veh = 1:length(Q)
    fprintf('Adding cylindrical obstacles vehicle %d for visualization...\n', veh)
    Q{veh} = addCylObs(Q{veh}, obj.g, rawCylObs);
  end
  
  % For saving graphics
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
end

%% Initialize figure
if save_png || save_fig
  f = figure;
  colors = lines(length(Q));
  plotTargetSets(Q, obj.g, colors)
  
  hc = cell(length(Q), 1); % Capture radius
  ho = cell(length(Q), 1); % Obstacle
  hn = cell(length(Q), 1); % Nominal trajectory
end

%% Simulate
for veh = 1:length(Q)
  Q{veh}.data.tauARmin = inf;
  Q{veh}.data.tauARmax = -inf;
end
tInds = cell(length(Q), 1);
for i = 1:length(tauAR)
  fprintf('t = %f\n', tauAR(i))
  
  %% Control and disturbance for SPP Vehicles
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInds{veh} = find(Q{veh}.data.nomTraj_tau > tauAR(i) - small & ...
      Q{veh}.data.nomTraj_tau < tauAR(i) + small, 1);
    
    if ~isempty(tInds{veh})
      Q{veh}.data.tauARmin = min(Q{veh}.data.tauARmin, tauAR(i));
      Q{veh}.data.tauARmax = max(Q{veh}.data.tauARmax, tauAR(i));
      liveness_rel_x = Q{veh}.data.nomTraj(:,tInds{veh}) - Q{veh}.x;
      liveness_rel_x(1:2) = rotate2D(liveness_rel_x(1:2), -Q{veh}.x(3));
      deriv = eval_u(RTTRS.g, RTTRS.Deriv, liveness_rel_x);
      
      u = RTTRS.dynSys.optCtrl([], liveness_rel_x, deriv, 'max');
      % Random disturbance
      d = Q{veh}.uniformDstb();
      Q{veh}.updateState(u, obj.dt, Q{veh}.x, d);
    end
  end
  
  %% Visualize
  if save_png || save_fig
    [hc, ho, hn] = plotVehicles(Q, tInds, obj.g, hc, ho, hn, colors, CARS.Rc);
    
    xlim([-1.2 1.2])
    ylim([-1.2 1.2])
    
    title(sprintf('t = %f', tauAR(i)))
    drawnow;
  end
  
  % Save plots
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
end

[Q1, Q2, Q3, Q4] = Q{:};
obj.tauAR = tauAR;
obj.tau = [obj.tauBR obj.tauAR];
obj.resim_filename = sprintf('resim_%f', now);
save(obj.resim_filename, 'Q1', 'Q2', 'Q3', 'Q4');
end