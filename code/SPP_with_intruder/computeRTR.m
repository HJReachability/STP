function computeRTR()
% Grid
grid_min = [-0.1; -0.1; -pi]; % Lower corner of computation domain
grid_max = [0.1; 0.1; pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

% Time
tMax = 1;
dt = 0.01;
tau = 0:dt:tMax;

% Vehicle
vRangeA = [0.1 1];
vReserved = [0.3 -0.3];

wMaxA = 1;
wReserved = -0.3;
dMaxA = [0.1 0.2];

% Virtual vehicle to be tracked
vRangeB = vRangeA + vReserved;
wMaxB = wMaxA + wReserved;
dMaxB = [0 0];
schemeData.dynSys = PlaneCAvoid( ...
  zeros(3,1), wMaxA, vRangeA, wMaxB, vRangeB, dMaxA, dMaxB);

% Initial conditions
trackingRadius = 0.075;
data0 = -shapeCylinder(schemeData.grid, 3, [ 0; 0; 0 ], trackingRadius);
schemeData.uMode = 'max';
schemeData.dMode = 'min';
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

save('RTR.mat', 'schemeData', 'data')
end