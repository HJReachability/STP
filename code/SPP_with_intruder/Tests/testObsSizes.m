function testObsSizes (initR, tIAT)

% This function computes the obstacle sizes after appending a FRS for tIAT
% time steps and then a backward reachable tube for tIAT time steps.

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% time vector
t0 = 0;
dt = 0.01;
tau = t0:dt:tIAT;

%% problem parameters
% Vehicle
speed = [0.5 1];
U = 1;
Rc = 0.1;
dMax = [0.1 0.2];

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!
schemeData.wMax = U;
schemeData.vrange = speed;
schemeData.dMax = dMax;
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';
schemeData.accuracy = 'medium';

% System dynamics
schemeData.hamFunc = @dubins3Dham;
schemeData.partialFunc = @dubins3Dpartial;

%% target set
% data0 = shapeCylinder(g, 3, [0; 0; 0], initR);
data0 = shapeSphere(g, [0; 0; 0], initR);

%% Compute the FRS
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];

tic  
dataFRS = HJIPDE_solve(data0, tau, schemeData, ...
  'none', extraArgs);
toc 

savename = sprintf('FRSObstacle_initR%d_tIAT%d', 10*initR, 10*tIAT);
grid on;
saveas(gca, savename , 'pdf')

%% Compute the BRS
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

% Base data now is to add the capture radius on the BRS
[g2D, data2D] = proj2D(g, dataFRS(:,:,:,end), [0,0,1]);
data2D = addCRadius(g2D, data2D, Rc);
data0_BRS = repmat(data2D, [1,1,g.shape(3)]);

dataBRS = HJIPDE_solve(data0_BRS, tau, schemeData,...
  'zero', extraArgs);

%% Save the results
savename = sprintf('finalObstacle_initR%d_tIAT%d', 10*initR, 10*tIAT);
grid on;
saveas(gca, savename , 'pdf')