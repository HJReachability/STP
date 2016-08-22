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
if exist(obj.BR_RS_filename_small, 'file')
  fprintf('Loading before-replanning RS...\n')
  load(obj.BR_RS_filename_small)
else
  error('Before-replanning RS file not found!')
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

% Determine time of simulation
Q = {Q1;Q2;Q3;Q4};
tStart = inf;
tEnd = -inf;
for veh = 1:length(Q)
  tStart = min(tStart, min(Q{veh}.data.nomTraj_tau));
  tEnd = max(tEnd, max(Q{veh}.data.nomTraj_tau));
end
tauBR = tStart:obj.dt:tEnd;

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
  
  intruder_color = 'k';
end

%% Initialize intruder
Qintr = Plane(intrIS, CARS.dynSys.wMaxB, CARS.dynSys.vRangeB, ...
  CARS.dynSys.dMaxB);

tReplan = inf;

safety_vals = 1e3*ones(length(Q), length(tauBR));
safety_threshold = 0.5;
intruder_arrived = false;

% Keep track of which vehicles need to replan
last_replan_veh = length(Q)+1;
for veh = 1:length(Q)
  Q{veh}.data.replan = false;
  Q{veh}.data.tauBRmin = inf;
  Q{veh}.data.tauBRmax = -inf;
end

%% Simulate
tInds = cell(length(Q),1);
safety_rel_x = cell(length(Q),1);

for i = 1:length(tauBR)
  if tauBR(i) > tReplan
    break
  end
  
  % tauBR(i) <= tReplan after this point
  fprintf('t = %f\n', tauBR(i)) 
  
  % Check if nominal trajectory has this t
  for veh = 1:length(Q)
    tInds{veh} = find(Q{veh}.data.nomTraj_tau > tauBR(i) - small & ...
      Q{veh}.data.nomTraj_tau < tauBR(i) + small, 1);
  end
  
  %% Intruder
  if tauBR(i) >= tIntr
    intrDstb = Qintr.uniformDstb();
    Qintr.updateState(intrCtrl, obj.dt, Qintr.x, intrDstb);
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
      tReplan = tauBR(i) + max(CARS.tau);
      intruder_arrived = true;
    end
  end
  
  %% Control and disturbance for SPP Vehicles
  for veh = 1:length(Q)
    if ~isempty(tInds{veh})
      Q{veh}.data.tauBRmin = min(Q{veh}.data.tauBRmin, tauBR(i));
      Q{veh}.data.tauBRmax = max(Q{veh}.data.tauBRmax, tauBR(i));
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
      Q{veh}.updateState(u, obj.dt, Q{veh}.x, d);
    end
  end
  
  %% Visualize
  if save_png || save_fig
    [hc, ho, hn] = plotVehicles(Q, tInds, obj.g, hc, ho, hn, colors, CARS.Rc);
    
    xlim([-1.2 1.2])
    ylim([-1.2 1.2])
    
    title(sprintf('t = %f', tauBR(i)))
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

%% Save data
fprintf('Saving data for replanning.\n')
for veh = last_replan_veh:length(Q)
  Q{veh}.data.replan = true;
end

Qintr.data.tauBR = obj.tIntr:obj.dt:obj.tReplan;

obj.tIntr = tIntr;
obj.tReplan = tauBR(i);
obj.tauBR = tauBR;

obj.BR_sim_filename = sprintf('%s_%f.mat', mfilename, now);

Qnew = cell(length(Q),1);
for veh = 1:length(Q)
  % Basic class properties
  Qnew{veh} = Plane(Q{veh}.x, Q{veh}.wMax, Q{veh}.vrange, Q{veh}.dMax);
  Qnew{veh}.xhist = Q{veh}.xhist;
  Qnew{veh}.u = Q{veh}.u;
  Qnew{veh}.uhist = Q{veh}.uhist;
  Qnew{veh}.hpxpy = Q{veh}.hpxpy;
  Qnew{veh}.hpxpyhist = Q{veh}.hpxpyhist;
  
  % Data
  Qnew{veh}.data.targetCenter = Q{veh}.data.targetCenter;
  Qnew{veh}.data.targetRsmall = Q{veh}.data.targetRsmall;
  Qnew{veh}.data.targetR = Q{veh}.data.targetR;
  Qnew{veh}.data.target = Q{veh}.data.target;
  Qnew{veh}.data.targetsm = Q{veh}.data.targetsm;
  
  Qnew{veh}.data.vReserved = Q{veh}.data.vReserved;
  Qnew{veh}.data.wReserved = Q{veh}.data.wReserved;
  
  Qnew{veh}.data.tauBR = Q{veh}.data.tauBRmin:obj.dt:Q{veh}.data.tauBRmax;
  Qnew{veh}.data.replan = Q{veh}.data.replan;
  
  % For vehicles that don't need to replan, copy over nominal trajectory
  if ~Q{veh}.data.replan
    Qnew{veh}.data.nomTraj = Q{veh}.data.nomTraj;
    Qnew{veh}.data.nomTraj_tau = Q{veh}.data.nomTraj_tau;
  end
end

[Q1, Q2, Q3, Q4] = Qnew{:};

save(obj.BR_sim_filename, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
end