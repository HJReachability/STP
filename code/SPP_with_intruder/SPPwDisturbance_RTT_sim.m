function SPPwDisturbance_RTT_sim(RTTRS_file, RTT_file, save_png, save_fig)

if nargin < 3
  save_png = true;
end

if nargin < 4
  save_fig = false;
end

% Simulation time
tMin = -3;
dt = 0.01;
tMax = 0;
tau = tMin:dt:tMax;

fprintf('Loading data files\n')
% Load robust tracking reachable set
load(RTTRS_file)

% Load path planning reachable set
load(RTT_file)
Q = {Q1;Q2;Q3;Q4};

% Gradient of RTTRS
fprintf('Computing gradient of RTTRS\n')
Deriv = computeGradients(RTTRS.g, RTTRS.data);

capture_radius = 0.1;
small = 1e-4;

% Plot targets
f = figure;
colors = lines(length(Q));
plotTargetSets(Q, schemeData, colors)

hc = cell(length(Q), 1);
ho = cell(length(Q), 1);

% For saving graphics
folder = sprintf('%s_%f', mfilename, now);
system(sprintf('mkdir %s', folder));

tInds = cell(length(Q), 1);
for i = 1:length(tau)
  fprintf('t = %f\n', tau(i))
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInds{veh} = find(Q{veh}.data.nomTraj_tau > tau(i) - small & ...
      Q{veh}.data.nomTraj_tau < tau(i) + small, 1);
    
    if ~isempty(tInds{veh})
      %% Get optimal control
      % Our plane is vehicle A, trying to stay out of reachable set, and the
      % reference virtual plane is vehicle B, trying to get into reachable set
      rel_x = Q{veh}.data.nomTraj(:,tInds{veh}) - Q{veh}.x;
      rel_x(1:2) = rotate2D(rel_x(1:2), -Q{veh}.x(3));
      
      deriv = eval_u(RTTRS.g, Deriv, rel_x);
      u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
      
      %% Get disturbance
      d = Q{veh}.uniformDstb();
      
      % Update state
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
    end
  end
  
  plotVehicles(Q, tInds, schemeData, hc, ho, colors, capture_radius)
  
  xlim([-1.2 1.2])
  ylim([-1.2 1.2])
  
  title(sprintf('t = %f', tau(i)))
  drawnow;
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
  
end

end