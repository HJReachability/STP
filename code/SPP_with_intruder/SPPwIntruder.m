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
Rc = 0.1; % Capture radius
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

filenames = {'SPPwIntruder_BRS1_1.mat'; ...
  'SPPwIntruder_BaseObs_1.mat'; ...
  'SPPwIntruder_AugObsFRS_1.mat'; ...
  'SPPwIntruder_FlatObs_1.mat'; ...
  'SPPwIntruder_AugObsBRS_1.mat'; ...
  'SPPwIntruder_BRS1_2.mat'; ...
  'SPPwIntruder_BaseObs_2.mat'; ...
  'SPPwIntruder_AugObsFRS_2.mat'; ...
  'SPPwIntruder_FlatObs_2.mat'; ...
  'SPPwIntruder_AugObsBRS_2.mat'; ...
  'SPPwIntruder_BRS1_3.mat'; ...
  'SPPwIntruder_BaseObs_3.mat'; ...
  'SPPwIntruder_AugObsFRS_3.mat'; ...
  'SPPwIntruder_FlatObs_3.mat'; ...
  'SPPwIntruder_AugObsBRS_3.mat'; ...
  'SPPwIntruder_BRS1_4.mat'};

for fileNum = 1:length(filenames)
  if exist(filenames{fileNum}, 'file')
    last_filename = filenames{fileNum};
  else
    break
  end
end

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if veh == 1
    obstacles = ones(N');
  end
  
  filename = sprintf('SPPwIntruder_BRS1_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    Q{veh} = computeBRS1(Q{veh}, tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    if strcmp(filename, last_filename)
      load(filename)
      Q = {Q1; Q2; Q3; Q4};
    end
  end
  
  %% Compute the base obstacles for based on BRS1
  filename = sprintf('SPPwIntruder_BaseObs_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    if veh < numVeh
      Q{veh} = computeBaseObs(Q{veh}, schemeData, resetR);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    if strcmp(filename, last_filename)
      load(filename)
      Q = {Q1; Q2; Q3; Q4};
    end
  end
  
  %% Augment base obstacles with t-IAT FRS
  filename = sprintf('SPPwIntruder_AugObsFRS_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    if veh < numVeh
      Q{veh} = augmentBaseObsFRS(Q{veh}, schemeData, tIAT);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    if strcmp(filename, last_filename)
      load(filename)
      Q = {Q1; Q2; Q3; Q4};
    end
  end
  
  %% Flatten augmented obstacles to 3D, add capture radius, and unflatten to 3D
  filename = sprintf('SPPwIntruder_FlatObs_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    if veh < numVeh
      Q{veh} = flatAugBOFRS(Q{veh}, schemeData, Rc);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    if strcmp(filename, last_filename)
      load(filename)
      Q = {Q1; Q2; Q3; Q4};
    end
  end
  
  %% Compute t-IAT backward reachable set from flattened 3D obstacle
  filename = sprintf('SPPwIntruder_AugObsBRS_%d.mat', veh);
  
  if restart || ~exist(filename, 'file')
    if veh < numVeh
      Q{veh} = augmentFlatObsBRS(Q{veh}, schemeData, tIAT);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'veh', '-v7.3')
  else
    if strcmp(filename, last_filename)
      load(filename)
      Q = {Q1; Q2; Q3; Q4};
    end
  end
end
end

%% Compute BRS1
function vehicle = computeBRS1(vehicle, tau, schemeData, obstacles)
% vehicle = computeBRS1(vehicle, tau, schemeData, obstacles)
%     Computes the first BRS for a vehicle, and updates its data with
%     BRS1_tau and BRS1 fields. This BRS is used for optimally getting to the
%     target despite the worst-case disturbances and moving obstacles induced by
%     other vehicles
%
% Inputs
%     vehicle:
%         vehicle for which the BRS is computed
%     tau:
%         the times for which the BRS is computed; unused times will be cut off
%     schemeData:
%         parameters for HJI PDE solver, should contain the following fields:
%         .dynSys, .grid
%     obstacles:
%         union of induced obstacles from higher-priority vehicles
%
% Output
%     vehicle:
%         updated vehicle object
%

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

%% computeBaseObs
function vehicle = computeBaseObs(vehicle, schemeData, resetR)
% vehicle = computeBaseObs(vehicle, schemeData, resetR)
%     Computes the induced base obstacles by vehicle according to BRS1 and the
%     centralized controller scheme; populates the .baseObs and .baseObs_tau
%     fields of vehicle.data
%
% Inputs
%     vehicle:
%         vehicle object; should have the .data field populated with .BRS1 and
%         .BRS1_tau
%     schemeData:
%         parameters for the HJI PDE solver; should contain the fields .grid and
%         .dynSys
%     resetR:
%         minimum size for the reachable set during evolution; if the reachable
%         set goes below this size, the reachable set will be propagated
%         "maximally" until the size becomes large enough
%
% Output
%     vehicle:
%         updated vehicle object with .baseObs and .baseObs_tau populated in the
%         vehicle.data field
%

% Set schemeData
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% Set computation time
tau = vehicle.data.BRS1_tau(1:end-1);

% Create a small obstacle around current vehicle
obs0 = genBaseObs0(schemeData.grid, vehicle.x, resetR);

% Set extraArgs for visualization
extraArgs.visualize = true;

% Set extraArgs for centralized controller
extraArgs.SDModFunc = @minIslandSize_SDFunc;
extraArgs.SDModParams.resetR = resetR;
extraArgs.SDModParams.BRS = vehicle.data.BRS1;
extraArgs.SDModParams.tau = vehicle.data.BRS1_tau;

colons = repmat({':'}, 1, schemeData.grid.dim);
extraArgs.obstacles = -vehicle.data.BRS1(colons{:}, 1:length(tau));

% Compute base obstacles
[vehicle.data.baseObs, vehicle.data.baseObs_tau] = ...
  HJIPDE_solve(obs0, tau, schemeData, 'none', extraArgs);

end

%% augmentBaseObsFRS
function vehicle = augmentBaseObsFRS(vehicle, schemeData, tIAT)
% vehicle = augmentBaseObsFRS(vehicle, schemeData, tIAT)
%     Augments the base obstacles by computing the tIAT-FRS from each obstacle
%     in time, and updates the vehicle object with fields .augObsFRS and
%     .augObsFRS_tau
%
% Inputs
%     vehicle:
%         vehicle object; must contains the fields .baseObs and .baseObs_tau in
%         the .data field
%     schemeData:
%         parameters for the HJI PDE solver; must contain the fields .grid and
%         .dynSys
%     tIAT:
%         maximum duration of the intruder
%
% Output
%     vehicle:
%         updated vehicle class; the fields .augObsFRS and .augObsFRS_tau will
%         be populated

% Extract the base obstacles
numObs = length(vehicle.data.baseObs_tau);

% Append each of the base obstacles by a tIAT step FRS
% Set schemeData
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% Set tau for computation
%   dt for computing augmentation; does not necessarily have to be the same as
%   the global dt
dt = 0.01;
tau = 0:dt:tIAT;

% Subtract the part of the obstacle that hits the target
extraArgs.obstacles = vehicle.data.target;

% Compute tIAT-FRS of every base obstacle
vehicle.data.augObsFRS = zeros(size(vehicle.data.baseObs));
for i = 1:numObs
  fprintf('Augmenting obstacle %d out of %d...\n', i, numObs)
  
  % Visualize the set every 10 time steps
  if ~mod(i-1, 10)
    extraArgs.visualize = true;
  else
    extraArgs.visualize = false;
  end
  
  augObsFRS = HJIPDE_solve(vehicle.data.baseObs(:, :, :, i), ...
    tau, schemeData, 'none', extraArgs);
  vehicle.data.augObsFRS(:,:,:,i) = augObsFRS(:, :, :, end);
  
  % Keep first FRS at every time step
  if i == 1
    augObsFRS_beginning = augObsFRS;
  end
end

% Shift time vector
vehicle.data.augObsFRS_tau = vehicle.data.baseObs_tau + tIAT;

% Add the obstacles in the beginning
vehicle.data.augObsFRS = ...
  cat(4, augObsFRS_beginning(:,:,:,1:end-1), vehicle.data.augObsFRS);
vehicle.data.augObsFRS_tau = ...
  [vehicle.data.baseObs_tau(1:(length(tau)-1)) vehicle.data.augObsFRS_tau];
end

%% Flatten obstacles
function vehicle = flatAugBOFRS(vehicle, schemeData, capture_radius)
% vehicle = flatAugBOFRS(vehicle, capture_radius)
%     Flattens and augments the tIAT-FRS-augmented obstacles, adds the capture
%     radius, and then extends the 2D shape into 3D. Populates the .flatObs2D
%     and .cylObs3D fields of vehicle.data
%

%% Parameters for adding capture radius
g2D = proj(schemeData.grid, [], [0 0 1]);

%% Initialize augmented-capture-radius flat obstacles
vehicle.data.flatObs2D = zeros([g2D.N' length(vehicle.data.augObsFRS_tau)]);
vehicle.data.cylObs3D = ...
  zeros([schemeData.grid.N' length(vehicle.data.augObsFRS_tau)]);

for i = 1:length(vehicle.data.augObsFRS_tau)
  [~, obs2D] = proj(schemeData.grid, vehicle.data.augObsFRS(:,:,:,i), [0 0 1]);
  vehicle.data.flatObs2D(:,:,i) = addCRadius(g2D, obs2D, capture_radius);
  vehicle.data.cylObs3D(:,:,:,i) = ...
    repmat(vehicle.data.flatObs2D(:,:,i), [1 1 schemeData.grid.N(3)]);
end
end

%% t-IAT BRS from flattened obstacles
function vehicle = augmentFlatObsBRS(vehicle, schemeData, tIAT)
% vehicle = augmentFlatObsBRS(vehicle, schemeData, tIAT)
%     Augments the flattened 3D obstacles with a t-IAT backward reachable set.
%     Populates the .augFlatObsBRS and .augFlatObsBRS_tau fields of vehicle.data
%


% Number of obstacles
numObs = length(vehicle.data.augObsFRS_tau);

% Set tau for computation; must have same dt as base obstacles
dt = diff(vehicle.data.augObsFRS_tau);
dt = dt(1);
tau = 0:dt:tIAT;

% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

% Compute tIAT-FRS of every base obstacle
vehicle.data.augFlatObsBRS = zeros(size(vehicle.data.augObsFRS));
for i = 1:numObs
  fprintf('Augmenting obstacle %d out of %d...\n', i, numObs)
  
  % Visualize the set every 10 obstacle augmentations
  if ~mod(i-1, 10)
    extraArgs.visualize = true;
  else
    extraArgs.visualize = false;
  end
  
  % The moving target is a union of all flattened obstacles up to time t
  extraArgs.targets = inf([schemeData.grid.N' length(tau)]);
  for j = 1:length(tau)
    if i-j+1 >= 1
      extraArgs.targets(:,:,:,j) = vehicle.data.cylObs3D(:,:,:,i-j+1);
    end
    
    extraArgs.targets(:,:,:,j) = min(extraArgs.targets(:,:,:,1:j), [], 4);
  end
  
  % Solve HJI PDE with the moving targets
  augFlatObsBRS = HJIPDE_solve(vehicle.data.cylObs3D(:,:,:,i), ...
    tau, schemeData, 'none', extraArgs);
  vehicle.data.augFlatObsBRS(:,:,:,i) = augFlatObsBRS(:, :, :, end);
  
end

% Shift time vector
vehicle.data.augFlatObsBRS_tau = vehicle.data.augObsFRS_tau - tIAT;

% Add last few obstacles
vehicle.data.augFlatObsBRS = ...
  cat(4, vehicle.data.augFlatObsBRS, augFlatObsBRS(2:end));
vehicle.data.augFlatObsBRS_tau = [vehicle.data.augFlatObsBRS_tau, ...
  vehicle.data.augObsFRS_tau(end-length(tau)+2:end)];
end