function SPPwIntruder_sim()
% Simulation, but for now just testing collision avoidance BRS computation

tMin = -3;
dt = 0.01;
tMax = 0;
tau = tMin:dt:tMax;


% Load robust tracking reachable set
load('RTTRS.mat')

% Load path planning reachable set
load('SPPwDisturbance_RS.mat')
Q = {Q1;Q2;Q3;Q4};

% Gradient of RTTRS
Deriv = computeGradients(RTTRS.g, RTTRS.data(:,:,:,end));

small = 1e-4;

% Plot targets
figure
colors = lines(length(Q));
plotTargetSets(Q, colors)

hc = cell(length(Q), 1);
ho = cell(length(Q), 1);

%% Initialize intruder
%--> Q_intruder = ...
tUpper = inf;

%% Load safety reachable set
load('CA.mat')
%--> safety_vf = ...


%% Safety values over time; each row is a vehicle, and each column is a time step
safety_vals = zeros(length(Q), length(tau));
safety_threshold = 0.5;
intruder_arrived = false;

%% Simulate
for i = 1:length(tau)
  % Move intruder
  %--> Q_intruder.updateState(
  
  % Hide intruder if time is later than tUpper
  if tau(i) > tUpper
    Q_intruder.unplotPosition();
  end
  
  % Check safety
  for veh = 1:length(Q)
    % Compute safety value
%     safety_vals(i,veh) = eval_u(g, safety_vf, Q_intruder.x-Q{veh}.x);
    
    if 0tau(i) <= tUpper
      %% Before intruder leaves airspace
      if safety_vals(i,veh) < safety_threshold
        % Safety controller
        % Record intruder's time of arrival and departure from the space
        if ~intruder_arrived
          tLower = tau(i);
          tUpper = tLower + tIAT;
          intruder_arrived = true;
        end
      else
        % Liveness controller
        %--> u = livenessCtrl();
        d = Q{veh}.GaussianDstb();
        Q{veh}.updateState(u, dt, Q{veh}.x, d);
      end
    else
      %% After intruder leaves airspace
      % Load or compute BRS2derivs
      %--> u = ...
      %--> d = ...
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
    end
  end
  
  % Plot vehicles
  plotVehicles([{Q_intruder}; Q]);
  
  % Plot obstacles
  plotObstacles(Q);
end