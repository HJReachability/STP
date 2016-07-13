function SPPwIntruder(restart)
% This function initializes the simulation for solving the SPP problem in
% the presence of intruder.

if nargin < 1
  restart = false;
end

%% Add the appropriate functions to the path
addpath(genpath('./obstacle_generation'));

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% Time parameters
t0 = 0;
tMax = 5;
dt = 0.01;
tIAT = 0.1;

% Set tau
tau = 0:dt:tMax;

%% Problem parameters
% Vehicle
vrange = [0.1 1];
wMax = 1;
Rc = 0.1;
dMax = [0.1 0.2];

% Intruder
speedI = [0.25 0.75];
UI = 0.5;

%% initial States
numVeh = 4;
Q = cell(numVeh,1);
Q{1} = Plane([-0.1; 0; 0], wMax, vrange, dMax);
Q{2} = Plane([ 0.1; 0; pi], wMax, vrange, dMax);
Q{3} = Plane([-0.1; 0.1; 7*pi/4], wMax, vrange, dMax);
Q{4} = Plane([ 0.1; 0.1; 5*pi/4], wMax, vrange, dMax);

%% target sets
R = 0.1;
R1 = 0.1;
Q{1}.data.target = shapeCylinder(g, 3, [0.7; 0.2; 0], R1);
Q{2}.data.target = shapeCylinder(g, 3, [-0.7; 0.2; 0], R1);
Q{3}.data.target = shapeCylinder(g, 3, [0.7; -0.7; 0], R1);
Q{4}.data.target = shapeCylinder(g, 3, [-0.7; -0.7; 0], R1);

% %% Reduced target set for the first BRS
% % R1 = 0.03;
% R1 = 0.1;
% Q{1}.target = shapeCylinder(g, 3, [0.7; 0.2; 0], R1);
% Q{2}.target = shapeCylinder(g, 3, [-0.7; 0.2; 0], R1);
% Q{3}.target = shapeCylinder(g, 3, [0.7; -0.7; 0], R1);
% Q{4}.target = shapeCylinder(g, 3, [-0.7; -0.7; 0], R1);

%% base obstacle data
% reset radius
resetR = [0.03, 0.03, 0.1]';

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!
schemeData.accuracy = 'medium';

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
  %% Step-1: Find out the set of obstacles induced by the higher priority
  % vehicles. It consists of three steps. In step-1a, we compute the
  % obstacles that correspond to the intruder being currently present in
  % the system. In step-2b we compute the obstacles that correspond the
  % case when intruder has already left the system. Finally, in step-2c, we
  % do the appropriate shifting of the obstacles and do union over all
  % vehicles. Since the computations are done recursively, we will only
  % compute the obstacles for the last vehicle.
  
  if veh ~= 1
    
    %% Step-1a: The obstcales should simply be given by the base obstacles
    % augmented by a tIAT-step FRS.
    
    % Extract the base obstacles
    obstacles = Q{veh-1}.Obs;
    numObs = size(obstacles, g.dim+1);
    
    % Append each of the base obstacles by a tIAT step FRS
    % Set schemeData
    schemeData.uMode = 'max';
    schemeData.dMode = 'max';
    schemeData.tMode = 'forward';
    
    % Set tau
    tau = 0:dt:tIAT;
    
    % Reset extraArgs
    extraArgs = [];
    
    % Set extraArgs
    extraArgs.visualize = false;
    extraArgs.plotData.plotDims = [1, 1, 0];
    extraArgs.plotData.projpt = Q{veh-1}.initState(3);
    
    % Subtract the part of the obstacle that hits the target
    extraArgs.obstacles = Q{veh-1}.data0;
    
    for i= 1:numObs
      % Visualize the set every 20 time steps
      if mod(i-1,20) == 0
        extraArgs.visualize = true;
      end
      
      [data, ~, ~] = HJIPDE_solve(obstacles(:, :, :, i), tau, schemeData,...
        'none', extraArgs);
      obstacles(:, :, :, i) = data(:, :, :, end);
      
      % Turn off the visualization
      extraArgs.visualize = false;
    end
    
    %% Step-1b: The obstcales can be computed in a moving target fashion
    % with the optimal control being given by the BRS2.
    
    % Set schemeData
    schemeDataBaseObs = schemeData;
    schemeDataBaseObs.uMode = 'max';
    schemeDataBaseObs.dMode = 'max';
    schemeDataBaseObs.tMode = 'forward';
    
    % System dynamics
    schemeDataBaseObs.hamFunc = @dubins3Dham_CCSObs;
    schemeDataBaseObs.partialFunc = @dubins3Dpartial;
    
    % Set tau
    % Obstacle is not computed at the last time step
    tau = tIAT: dt: Q{veh-1}.tau_BRS2(end-1);
    
    % Set extraArgs
    extraArgs = [];
    extraArgs.visualize = true;
    extraArgs.plotData.plotDims = [1, 1, 0];
    extraArgs.plotData.projpt = Q{veh-1}.initState(3);
    
    % Compute the time and the correspodning index at which the obstcales
    % corresponding to step-1b will start appear.
    tStart = tIAT;
    tStart_ind = find(Q{veh-1}.tau_BRS2 == tStart);
    
    extraArgs.genparams.data = Q{veh-1}.data_BRS2(:, :, :, end: -1: ...
      tStart_ind);
    extraArgs.genparams.reset_thresholds = resetR;
    
    % There won't be any targets to add once the base obstacles end up
    numEmpTargets = length(tau) - numObs;
    emptyTargets = repmat(ones(g.shape), [ones(1,g.dim) numEmpTargets]);
    extraArgs.targets = cat(g.dim+1, obstacles, emptyTargets);
    
    [data, tau, ~] = computeCCSObs(obstacles(:, :, :, 1), tau, ...
      schemeDataBaseObs, 'none', extraArgs);
    
    %% Step-1c: Compute the overall obstacles.
    % Starting at time tIAT the obstacle sequence is simply given by the
    % data above in step-1b. Before that let's just use the first obstacle
    % itself.
    
    % For the tIAT-1 time just use the first obstacle
    obsShift = repmat(obstacles(:, :, :, 1), [ones(1,g.dim) tStart_ind-1]);
    
    % Now we are ready for the overall obstacles
    obstacles = cat(g.dim+1, obsShift, data);
    
    % Add capture radius to obstacles
    numObs = size(obstacles, g.dim+1);
    for i= 1:numObs
      [g2D, data2D] = proj2D(g, obstacles(:, :, :, i), [0,0,1]);
      data2D = addCRadius(g2D, data2D, Rc);
      obstacles(:, :, :, i) = repmat(data2D, [1,1,g.shape(3)]);
    end
    
    % Assign this obstacle sequence to the vehicle
    Q{veh-1}.Obs = obstacles;
    
    % Convert the obstacle set to a fixed time-scale of [0, tMax] so that
    % the union makes sense
    fixedtau = 0:dt:tMax;
    fixedScaleObs = repmat(ones(g.shape), [ones(1,g.dim) length(fixedtau)]);
    fixedScaleObs(:, :, :, end-numObs:end-1) = obstacles;
    
    % Overload the obstacles with these new obstacles
    obstacles = fixedScaleObs;
    numObs = size(obstacles, g.dim+1);
    
    % Finally, take union of obstacles across all vehicles to get the overall
    % obstacle set
    if exist('unionObs', 'var')
      for i=1:numObs
        unionObs(:, :, :, i) = shapeUnion(unionObs(:, :, :, i), ...
          obstacles(:, :, :, i));
      end
    else
      unionObs = obstacles;
    end
    
    % Save the sets, just in case
    filename = sprintf('SPPwIntruder_check1_%d', veh);
    var2save = Q{veh-1};
    save(filename, 'var2save', 'unionObs', '-v7.3')
  end
  
  %% Step-2a: Augment the obstacles by a tIAT step BRS
  % It should be simply the obstacles in Step1 augmented by a tIAT-step
  % BRT and then union over all time steps for the duraction [0 tIAT].
  if veh ~= 1
    
    % Append the obstacle i-step BRS of the obstacle corresponding to time
    % t0-i, where t0 is the current time.
    % Reset the obstacle matrix
    obstacles = [];
    
    % Set schemeData
    schemeData.uMode = 'min';
    schemeData.dMode = 'min';
    schemeData.tMode = 'backward';
    
    % Set tau
    tau = 0:dt:tIAT;
    
    % Reset extraArgs
    extraArgs = [];
    
    % Set extraArgs
    extraArgs.visualize = false;
    extraArgs.plotData.plotDims = [1, 1, 0];
    extraArgs.plotData.projpt = Q{veh-1}.initState(3);
    
    % Number of obstacles
    numObs = size(unionObs, g.dim+1);
    
    for i= 1:numObs-1
      % Visualize the set every 20 time steps
      if mod(i-1,20) == 0
        extraArgs.visualize = true;
      end
      
      % Set the moving targets
      shifts = floor((tIAT - t0)/dt);
      if i+shifts <= numObs
        targets = unionObs(:, :, :, i+shifts:-1:i);
      else
        tend = (numObs - i)*dt;
        tau = 0: dt: tend;
        targets = unionObs(:, :, :, numObs:-1:i);
      end
      extraArgs.targets = targets;
      
      [data, ~, ~] = HJIPDE_solve(targets(:, :, :, 1), tau, schemeData,...
        'zero', extraArgs);
      obstacles(:, :, :, i) = data(:, :, :, end);
      
      % By default turn off the visualization
      extraArgs.visualize = false;
    end
    obstacles(:, :, :, numObs) = unionObs(:, :, :, numObs);
    
    % Save the sets, just in case
    filename = sprintf('SPPwIntruder_check2_%d', veh);
    save(filename, 'obstacles', '-v7.3')
  end
  
  %% Step-2b: Compute the BRS of the vehicle with the above obstacles
  if veh == 1
    obstacles = ones(N');
  end
  
  filename = sprintf('SPPwIntruder_BRS1_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    Q{veh} = computeBRS1(Q{veh}, tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    load(filename)
    Q = {Q1; Q2; Q3; Q4};
  end
  
  %% Step-3c: Compute the base obstacles for vehicles
  filename = sprintf('SPPwIntruder_BaseObs_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    if veh < numVeh
      Q{veh} = computeBaseObs(Q{veh}, schemeData, resetR);
    end
   
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    load(filename)
    Q = {Q1; Q2; Q3; Q4};
  end
end
end
%%
function vehicle = computeBRS1(vehicle, tau, schemeData, obstacles)
% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% Set extraArgs
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = vehicle.x(3);

% Set obstacles
extraArgs.obstacles = obstacles;

% Computation should stop once it contains the initial state
extraArgs.stopInit = vehicle.x;

[vehicle.data.BRS1, vehicle.data.BRS1_tau] = ...
  HJIPDE_solve(vehicle.data.target, tau, schemeData, 'zero', extraArgs);

t0 = vehicle.data.BRS1_tau(1);
vehicle.data.BRS1_tau = 2*t0 - vehicle.data.BRS1_tau;

% Reverse the order of time elements
vehicle.data.BRS1_tau = flip(vehicle.data.BRS1_tau);
vehicle.data.BRS1 = flip(vehicle.data.BRS1, 4);
end

%%
function vehicle = computeBaseObs(vehicle, schemeData, resetR)
% Set schemeData
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% Set computation time
tau = vehicle.data.BRS1_tau(1:end-1);

% Create a small obstacle around current vehicle
obs0 = genBaseObs0(schemeData.grid, vehicle.x, resetR);

% Set extraArgs
extraArgs.visualize = true;

% Compute base obstacles
[vehicle.data.baseObs, vehicle.data.baseObs_tau] = ...
  HJIPDE_solve(obs0, tau, schemeData, 'none', extraArgs);

end