function SPPwIntruder_sim(RTTRS_filename, RS_filename, CA_filename, ...
  AI_filename, savepng)

if nargin < 5
  savepng = true;
end

small = 1e-4;

tMin = -3;
dt = 0.01;
tMax = 0;
tau = tMin:dt:tMax;

%% Load robust tracking reachable set
load(RTTRS_filename)

% Gradient of RTTRS
RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data(:,:,:,end));

%% Load path planning reachable set
load(RS_filename)
Q = {Q1;Q2;Q3;Q4};

% Plot targets
figure
colors = lines(length(Q));
plotTargetSets(Q, colors)

hc = cell(length(Q), 1);
ho = cell(length(Q), 1);

%% Initialize intruder
% Load safety reachable set
load(CA_filename)
CARS.Deriv = computeGradients(CARS.g, CARS.data(:,:,:,end));

Q_intruder = Plane( ...
  [0; -0.5; -pi], CARS.dynSys.wMaxB, CARS.dynSys.vRangeB, CARS.dynSys.dMaxB);

intruder_color = 'k';
tUpper = inf;

safety_vals = nan(length(Q), length(tau));
safety_threshold = 0.5;
intruder_arrived = false;

% Intruder control
u_intruder = [0.5; 0];

%% Simulate
tInds = zeros(length(Q),1);
safety_rel_x = cell(length(Q),1);
for i = 1:length(tau)
  %% Intruder
  d_intruder = Q_intruder.uniformDstb();
  Q_intruder.updateState(u_intruder, dt, Q_intruder.x, d_intruder);
  
  % Hide intruder if time is later than tUpper
  if tau(i) > tUpper
    Q_intruder.unplotPosition();
  else
    Q_intruder.plotPosition(intruder_color);
  end
  
  % Check safety
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInds(veh) = find(Q{veh}.data.nomTraj_tau > tau(i) - small & ...
      Q{veh}.data.nomTraj_tau < tau(i) + small, 1);
    
    % Compute safety value
    if ~isempty(tInds(veh))
      safety_rel_x{veh} = Q_intruder.x - Q{veh}.x;
      safety_rel_x{veh}(1:2) = rotate2D(safety_rel_x{veh}(1:2), -Q{veh}.x(3));
      safety_vals(veh, i) = eval_u(g, safety_vf, safety_rel_x{veh});
    else
      safety_vals(veh, i) = 1e3;
    end
  end
  
  % Mark time at which intruder shows up
  if ~intruder_arrived && any(safety_vals(:, i) < safety_threshold)
    tLower = tau(i);
    tUpper = tLower + tIAT;
    intruder_arrived = true;
  end
  
  %% Control and disturbance for SPP Vehicles
  if tau(i) <= tUpper
    for veh = 1:length(Q)
      if ~isempty(tInds(veh))
        if safety_vals(veh, i) < safety_threshold
          % Safety controller
          u = CARS.dynSys.optCtrl([], safety_rel_x{veh}, CARS.Deriv, 'max');
        else
          liveness_rel_x = Q{veh}.data.nomTraj(:,tInd) - Q{veh}.x;
          liveness_rel_x(1:2) = rotate2D(liveness_rel_x(1:2), -Q{veh}.x(3));
          u = RTTRS.dynSys.optCtrl([], liveness_rel_x, deriv, 'max');
        end
        % Random disturbance
        d = Q{veh}.GaussianDstb();
      end
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
    end
  else
    %% Control after intruder leaves airspace
    % Load or compute BRS2derivs
    if ~exist(AI_filename, 'file')
      currentStates = cell(length(Q), 1);
      targetCenters = cell(length(Q), 1);
      for veh = 1:length(Q)
        currentStates{veh} = Q{veh}.x;
        targetCenters{veh} = Q{veh}.data.targetCenter;
      end
      SPPwIntruder_replan_RS(RTTRS_filename, true, AI_filename, Q);
    end
    
    fprintf('Done for now')
    return
%     load(AI_filename)
    
    
    %--> u = ...
    %--> d = ...
    
  end
  
  % Plot vehicles
  for veh = 1:length(Q)
    % Plot capture radius
    if isempty(hc{veh})
      hc{veh} = plotDisk( ...
        Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
    else
      [~, hc{veh}.XData, hc{veh}.YData] = plotDisk( ...
        Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
    end    
  end
end
end