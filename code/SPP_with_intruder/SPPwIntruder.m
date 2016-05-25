% This function initializes the simulation for solving the SPP problem in
% the presence of intruder. 

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

% Base grid - it will be used for computation of recahable sets by union= 
Nfine = [151; 151; 151];
base_g = createGrid(grid_min, grid_max, Nfine, pdDims);

%% time vector
t0 = 0;
tMax = 4;
dt = 0.01;
tIAT = 0.2;

%% problem parameters
% Vehicle
speed = [0.5 1];
U = 1;
Rc = 0.1;
dMax = [0.1 0.2];

% Intruder
speedI = [0.25 0.75]; 
UI = 0.5;

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!
schemeData.wMax = U;
schemeData.vrange = speed;
schemeData.dMax = dMax;
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% System dynamics
schemeData.hamFunc = @dubins3Dham;
schemeData.partialFunc = @dubins3Dpartial;

%% target set
R = 0.1;
Q{1}.data0 = shapeCylinder(g, 3, [0.7; 0.2; 0], R);
Q{2}.data0 = shapeCylinder(g, 3, [-0.7; 0.2; 0], R);
Q{3}.data0 = shapeCylinder(g, 3, [0.7; -0.7; 0], R);
Q{4}.data0 = shapeCylinder(g, 3, [-0.7; -0.7; 0], R);

%% Compute the base FRS  
filename = ['baseBRS_' num2str(schemeData.wMax) ...
  '_' num2str(max(schemeData.vrange)) '.mat'];

if exist(filename, 'file')
  load(filename)
else
  tau = t0:dt:tIAT;
  schemeDataFine = schemeData;
  schemeData.uMode = 'max';
  schemeData.dMode = 'max';
  schemeData.tmode = 'forward';
  schemeDataFine.grid = base_g;
  base_data0 = shapeRectangleByCorners(base_g, -g.dx/2, g.dx/2);
  extraArgs.plotData.plotDims = [1, 1, 0];
  extraArgs.plotData.projpt = [0];
  
  base_data = HJIPDE_solve(base_data0, tau, schemeDataFine, ...
    'zero', extraArgs);
  save(filename, 'base_g', 'base_data', '-v7.3')
end

%% Reset extraArgs
extraArgs = [];

%% Step-1: Find out the set of obstacles induced by the higher priority
% vehicles. It should simply the base obstacles augmented by a tIAT-step
% FRS and then union over all vehicles. Since the computations are done
% recursively, we will only compute the obstacles for the last vehicle.

if veh ~= 1
  
  % Extract the base obstacles
  obstacles = Q{veh-1}.Obs;
  obstau = Q{veh-1}.Obstau;
  numObs = size(obstacles, g.dim+1);
  
  % Append the base obstacles by a tIAT step FRS
  for i= 1:numObs
    obstacles(:, :, :, i) = computeDataByUnion(base_g, ...
      base_data(:, :, :, end), g, obstacles(:, :, :, i));
  end
  
  % Shift the obstacles sequence forward by tIAT to get the correct
  % obstacle sequence
  shifts = floor((tIAT - t0)/dt);
  % For the tIAT time just use the first obstacle 
  obstacles(:, :, :, 1:shifts) = repmat(obstacles(:, :, :, 1), ...
    [ones(1,g.dim) shifts]);
  % Thereafter shift the obstacles by tIAT 
  obstacles(:, :, :, shifts+1:end) = obstacles(:, :, :, 1:end-shifts);
  
  % Assign this obstacle sequence to the vehicle
  Q{veh-1}.Obs = obstacles;
  
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
  
end 
  

%% Step-2: Augment the obstacles by a tIAT step BRS
% It should simply the base obstacles augmented by a tIAT-step
% FRS and then union over all vehicles. Since the computations are done
% recursively, we will only compute the obstacles for the last vehicle.

  
  
  
  