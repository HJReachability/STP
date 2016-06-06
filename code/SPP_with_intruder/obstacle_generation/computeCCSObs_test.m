function computeCCSObs_test()

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% time vector
t0 = 0;
tMax = 1;
dt = 0.01;

%% problem parameters
% Vehicle
speed = [0.5 1];
U = 1;
dMax = [0.1 0.2];

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!
schemeData.wMax = U;
schemeData.vrange = speed;
schemeData.dMax = dMax;
schemeData.accuracy = 'medium';

% Initial state and reset radius
initState = [-0.5, 0, 0]';
resetR = [0.03, 0.03, 0.1]';

%% Compute the BRS from the target
R = 0.1;
data0 = shapeCylinder(g, 3, [0; 0; 0], R);

% Set tau 
tau = t0:dt:tMax;

% Problem parameters
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% System dynamics
schemeData.hamFunc = @dubins3Dham;
schemeData.partialFunc = @dubins3Dpartial;

% Set extraArgs
extraArgs = [];
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];
extraArgs.stopInit = initState;

[data, tau, ~] = HJIPDE_solve(data0, tau, schemeData,...
  'zero', extraArgs);

%% Compute the base obstacles
% Problem parameters
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% System dynamics
schemeData.hamFunc = @dubins3Dham_CCSObs;
schemeData.partialFunc = @dubins3Dpartial;

% Set extraArgs
extraArgs = [];
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];
extraArgs.genparams.data = data(:, :, :, end:-1:1);
extraArgs.genparams.reset_thresholds = resetR;

obs0 = genBaseObs0(g, initState, resetR);

[data, tau, ~] = computeCCSObs(obs0, tau, schemeData,...
  'none', extraArgs);

numObs = length(tau);
figure,
for i=1:numObs
  [gProj, dataProj] = proj(g, data(:, :, :, i), ...
    [0, 0, 1], 0);
  hT = visSetIm(gProj, dataProj, 'r', 0, [], false);
  pause(1);
end