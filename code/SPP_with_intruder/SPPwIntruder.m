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
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

%% Time parameters
t0 = 0;
% Maximum time-horizon of computation (t = -tMax is the minimum absolute time)
tMax = 5;
% Set tau
dt = 0.01;
tIAT = 0.1;
tau = t0:dt:tMax;

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
Q{1}.data.target = shapeCylinder(schemeData.grid, 3, [0.7; 0.2; 0], R1);
Q{2}.data.target = shapeCylinder(schemeData.grid, 3, [-0.7; 0.2; 0], R1);
Q{3}.data.target = shapeCylinder(schemeData.grid, 3, [0.7; -0.7; 0], R1);
Q{4}.data.target = shapeCylinder(schemeData.grid, 3, [-0.7; -0.7; 0], R1);

% %% Reduced target set for the first BRS
% % R1 = 0.03;
% R1 = 0.1;
% Q{1}.target = shapeCylinder(g, 3, [0.7; 0.2; 0], R1);
% Q{2}.target = shapeCylinder(g, 3, [-0.7; 0.2; 0], R1);
% Q{3}.target = shapeCylinder(g, 3, [0.7; -0.7; 0], R1);
% Q{4}.target = shapeCylinder(g, 3, [-0.7; -0.7; 0], R1);

%% Reset radius for base obstacle computation
resetR = [0.03, 0.03, 0.1]';

%% File names for saving

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
  end
end

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};

  %% Gather induced obstacles of higher-priority vehicles
  % Assume there's no static obstacle
  obstacles = gatherObstacles(Q{1:veh-1}, schemeData, tau);
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
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