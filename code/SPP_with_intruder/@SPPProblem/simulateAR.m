function simulateAR(obj, save_png, save_fig)
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
if exist(obj.rawAugObs_filename, 'file')
  fprintf('Loading ''raw'' augmented obstacles...\n')
  load(obj.rawAugObs_filename)
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
  tEnd = max(tEnd, max(Q{veh}.nomTraj_tau));
end
tauAR = obj.tReplan:obj.dt:tEnd;

% Add cylindrical obstacles for visualization
if save_png || save_fig
  for veh = 1:length(Q)
    fprintf('Adding obstacles vehicle %d for visualization...\n', veh)
    Q{veh}.addObs2D(obj, RTTRS, CARS);
  end
  
  % For saving graphics
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
end

%% Initialize figure
if save_png || save_fig
  f = figure;
  colors = lines(length(Q));
  plotTargetSets(Q, colors)
  
  hc = cell(length(Q), 1); % Capture radius
  ho = cell(length(Q), 1); % Obstacle
  hn = cell(length(Q), 1); % Nominal trajectory
end

%% Simulate
tInds = cell(length(Q),1);
tauARmin = inf(length(Q), 1);
tauARmax = -inf(length(Q), 1);

for i = 1:length(tauAR)
  fprintf('t = %f\n', tauAR(i))
  
  %% Control and disturbance for SPP Vehicles
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInds{veh} = find(Q{veh}.nomTraj_tau > tauAR(i) - small & ...
      Q{veh}.nomTraj_tau < tauAR(i) + small, 1);
    
    if ~isempty(tInds{veh})
      tauARmin(veh) = min(tauARmin(veh), tauAR(i));
      tauARmax(veh) = max(tauARmax(veh), tauAR(i));
      liveness_rel_x = Q{veh}.nomTraj(:,tInds{veh}) - Q{veh}.x;
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
    [hc, ho, hn] = plotVehicles(Q, tInds, obj.g2D, hc, ho, hn, colors, CARS.Rc);
    
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

for veh = 1:length(Q)
  Q{veh}.tauAR = tauARmin(veh):obj.dt:tauARmax(veh);
  Q{veh}.tau = [Q{veh}.tauBR Q{veh}.tauAR];
end
[Q1, Q2, Q3, Q4] = Q{:};

obj.tauAR = tauAR;
obj.tau = [obj.tauBR obj.tauAR];
obj.full_sim_filename = sprintf('resim_%f.mat', now);
save(obj.full_sim_filename, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr');
end