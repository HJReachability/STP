function SPPwIntruder_sim()
% Simulation, but for now just testing collision avoidance BRS computation

%% Problem parameters
% Vehicle
vRangeA = [0.1 1];
wMaxA = 1;
Rc = 0.1; % Capture radius
dMaxA = [0.1 0.2];

% Intruder
vRangeB = vRangeA;
wMaxB = wMaxA;
dMaxB = dMaxA;

schemeData.dynSys = ...
  PlaneCAvoid([0; 0; 0], wMaxA, vRangeA, wMaxB, vRangeB, dMaxA, dMaxB);
schemeData.uMode = 'max';
schemeData.dMode = 'min';

%% Grid and target set
grid_min = [-0.15; -0.25; 0]; % Lower corner of computation domain
grid_max = [0.25; 0.25; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

data0 = shapeCylinder(schemeData.grid, 3, [0; 0; 0], Rc);

%% Time stamps
t0 = 0;
dt = 0.01;
tIAT = 0.1;
tau = t0:dt:tIAT;

%% Compute set
extraArgs.visualize = true;
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
save('CA.mat', 'schemeData', 'data')

end