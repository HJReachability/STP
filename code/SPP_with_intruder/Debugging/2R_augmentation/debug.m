%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% time vector
dt = 0.01;
tIAT = 0.1;

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
schemeData.w1Max = U;
schemeData.w2Max = UI;
schemeData.vrange1 = speed;
schemeData.vrange2 = speedI;

schemeData.dMax = dMax;
schemeData.accuracy = 'medium';
schemeData.PEmode = '2Rcal';

% System dynamics
schemeData.hamFunc = @dubins3DPEham_2Rcal;
schemeData.partialFunc = @dubins3DPEpartial_2Rcal;

%% target sets
R = 0.2;
data0 = shapeCylinder(g, 3, [0; 0; 0], R);

% Set tau
tau = 0:dt:2*tIAT;

% Reset extraArgs
extraArgs = [];

% Set extraArgs
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [pi];

[data, tau, ~] = HJIPDE_solve(data0, tau, schemeData,...
  'zero', extraArgs);