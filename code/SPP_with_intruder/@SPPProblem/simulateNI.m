function simulateNI(obj, save_png, save_fig)
% simulateNI(obj, save_png, save_fig)
%     Simulates SPP with disturbances with the RTT method

if nargin < 2
  save_png = true;
end

if nargin < 3
  save_fig = false;
end

%% Load files
% Load robust tracking reachable set
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

% Load path planning reachable set
if exist(obj.NI_RS_filename_small, 'file')
  fprintf('Loading RS data...\n')
  load(obj.NI_RS_filename_small)
else
  error('RS file not found!')
end

%% Post process loaded data
% Compute gradients used for optimal control
fprintf('Computing gradients...\n')
RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data);

% Determine time of simulation
Q = {Q1;Q2;Q3;Q4};
tStart = inf;
tEnd = -inf;
for veh = 1:length(Q)
  Q{veh}.x = Q{veh}.xhist(:,1);
  Q{veh}.xhist = Q{veh}.x;
  Q{veh}.u = [];
  Q{veh}.uhist = [];
  tStart = min(tStart, min(Q{veh}.nomTraj_tau));
  tEnd = max(tEnd, max(Q{veh}.nomTraj_tau));
end
tau = tStart:obj.dt:tEnd;

% Add cylindrical obstacles for visualization
if save_png || save_fig
  for veh = 1:length(Q)
    fprintf('Adding obstacles for vehicle %d for visualization...\n', veh)
    Q{veh}.addObs2D(obj, RTTRS);
  end
  
  % For saving graphics
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
  
  % Initialize figure
  f = figure;
  colors = lines(length(Q));
  plotTargetSets(Q, colors)
  
  hc = cell(length(Q), 1); % Capture radius
  ho = cell(length(Q), 1); % Obstacle
  hn = cell(length(Q), 1); % Nominal trajectory
end

small = 1e-4;
tInds = cell(length(Q), 1);
for i = 1:length(tau)
  fprintf('t = %f\n', tau(i))
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInds{veh} = find(Q{veh}.nomTraj_tau > tau(i) - small & ...
      Q{veh}.nomTraj_tau < tau(i) + small, 1);
    
    if ~isempty(tInds{veh})
      %% Get optimal control
      % Our plane is vehicle A, trying to stay out of reachable set, and the
      % reference virtual plane is vehicle B, trying to get into reachable set
      rel_x = Q{veh}.nomTraj(:,tInds{veh}) - Q{veh}.x;
      rel_x(1:2) = rotate2D(rel_x(1:2), -Q{veh}.x(3));
      
      deriv = eval_u(RTTRS.g, RTTRS.Deriv, rel_x);
      u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
      
      %% Get disturbance
      d = Q{veh}.uniformDstb();
      
      % Update state
      Q{veh}.updateState(u, obj.dt, Q{veh}.x, d);
    end
  end
  
  %% Visualize
  if save_png || save_fig
    [hc, ho, hn] = plotVehicles(Q, tInds, obj.g2D, hc, ho, hn, colors, obj.Rc);
    
    xlim([-1.2 1.2])
    ylim([-1.2 1.2])
    
    title(sprintf('t = %f', tau(i)))
    drawnow;
  end
  
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
end

obj.NI_sim_filename = sprintf('%s_%f.mat', mfilename, now);
[Q1, Q2, Q3, Q4] = Q{:};
save(obj.NI_sim_filename, 'Q1', 'Q2', 'Q3', 'Q4', '-v7.3')

SPPP = obj;
save(obj.this_filename, 'SPPP', '-v7.3')
end