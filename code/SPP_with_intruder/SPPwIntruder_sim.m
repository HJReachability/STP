function SPPwIntruder_sim(RTTRS_filename, RS_filename, CA_filename, ...
  save_png, save_fig)

if nargin < 5
  save_png = true;
end

if nargin < 6
  save_fig = false;
end

capture_radius = 0.1;
small = 1e-4;

tMin = -3;
dt = 0.01;
tMax = 0;
tau = tMin:dt:tMax;

tIAT = 0.1;

%% Load robust tracking reachable set
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)

% Gradient of RTTRS
RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data);

%% Load path planning reachable set
fprintf('Loading main RS...\n')
load(RS_filename)
Q = {Q1;Q2;Q3;Q4};

% Plot targets
figure
colors = lines(length(Q));
plotTargetSets(Q, schemeData, colors)

hc = cell(length(Q), 1);
ho = cell(length(Q), 1);

% Add cylindrical obstacles for visualization
fprintf('Loading ''raw'' obstacles...\n')
load(Obs_filename)
rawCylObs.data = zeros([schemeData.grid.N' length(tauIAT)]);
for i = 1:length(tauIAT)
  rawCylObs.data(:,:,:,i) = ...
    migrateGrid(rawObs.g, rawObs.cylObs3D(:,:,:,i), schemeData.grid);
end
rawCylObs.tauIAT = rawObs.tauIAT;

for veh = 1:length(Q)
  Q{veh} = addCylObs(Q{veh}, schemeData, rawCylObs);
end

% For saving graphics
folder = sprintf('%s_%f', mfilename, now);
system(sprintf('mkdir %s', folder));

%% Initialize intruder
% Load safety reachable set
fprintf('Loading CARS...\n')
load(CA_filename)
CARS.Deriv = computeGradients(CARS.g, CARS.data);

Q_intruder = Plane( ...
  [0; -0.5; pi/2], CARS.dynSys.wMaxB, CARS.dynSys.vRangeB, CARS.dynSys.dMaxB);

intruder_color = 'k';
tLower = -2.5;
tUpper = inf;

safety_vals = 1e3*ones(length(Q), length(tau));
safety_threshold = 0.5;
intruder_arrived = false;

% Intruder control
u_intruder = [0.5; 0];

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
  if tau(i) >= tLower && tau(i) <= tUpper
    d_intruder = Q_intruder.uniformDstb();
    Q_intruder.updateState(u_intruder, dt, Q_intruder.x, d_intruder);
    Q_intruder.plotPosition(intruder_color);

    % Check safety
    for veh = 1:length(Q)
      % Compute safety value
      if ~isempty(tInds{veh})
        safety_rel_x{veh} = Q_intruder.x - Q{veh}.x;
        safety_rel_x{veh}(1:2) = rotate2D(safety_rel_x{veh}(1:2), -Q{veh}.x(3));
        safety_vals(veh, i) = eval_u(CARS.g, CARS.data, safety_rel_x{veh});
      end
    end
    
    % Mark time at which intruder shows up
    if ~intruder_arrived && any(safety_vals(:, i) < safety_threshold)
      tUpper = tau(i) + tIAT;
      intruder_arrived = true;
    end
  elseif tau(i) > tUpper
    % Hide intruder if time is later than tUpper
    Q_intruder.unplotPosition();
  end
  
  %% Control and disturbance for SPP Vehicles
  if tau(i) <= tUpper
    for veh = 1:length(Q)
      if ~isempty(tInds{veh})
        if safety_vals(veh, i) < safety_threshold
          fprintf('Vehicle %d is performing avoidance.\n', veh)
          % Safety controller
          deriv = eval_u(CARS.g, CARS.Deriv, safety_rel_x{veh});
          u = CARS.dynSys.optCtrl([], safety_rel_x{veh}, deriv, 'max');
          
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
  else
    fprintf('Saving data for replanning.\n')
    saveReplanData(Q, schemeData, tau(i));
    return
  end
  
  % Visualize
  [hc, ho] = plotVehicles(Q, tInds, schemeData, hc, ho, colors, capture_radius);
  
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