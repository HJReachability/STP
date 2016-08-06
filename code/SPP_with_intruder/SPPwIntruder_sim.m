function SPPwIntruder_sim()
% Simulation, but for now just testing collision avoidance BRS computation

% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

% Time
%--> tau = ...

%% Load vehicles with gradient of BRS1 in the data field
%--> load ...

%% Initialize intruder
%--> Q_intruder = ...
tUpper = inf;

%% Load safety reachable set
%--> load ...
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
  for j = 1:length(Q)
    % Compute safety value
    safety_vals(i,j) = eval_u(g, safety_vf, Q_intruder.x-Q{j}.x);
    
    if tau(i) <= tUpper
      %% Before intruder leaves airspace
      if safety_vals(i,j) < safety_threshold
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
        %--> d = ...
        Q{j}.updateState(u, dt, Q{j}.x, d);
      end
    else
      %% After intruder leaves airspace
      % Load or compute BRS2derivs
      %--> u = ...
      %--> d = ...
      Q{j}.updateState(u, dt, Q{j}.x, d);
    end
  end
  
  % Plot vehicles
  plotVehicles([{Q_intruder}; Q]);
  
  % Plot obstacles
  plotObstacles(Q);
end