function computeCARS(tIAT, save_png)
% Just a simple file for computing collision avoidance reachable set...

if nargin < 1
  tIAT = 0.25;
end

if nargin < 2
  save_png = true;
end

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
tau = t0:dt:tIAT;

%% Compute set
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

if save_png
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
  extraArgs.fig_filename = sprintf('%s/', folder);
end
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

CARS.dynSys = schemeData.dynSys;
CARS.g = schemeData.grid;
CARS.data = data(:,:,:,end);
CARS.tau = tau;
save(sprintf('CARS_%f.mat', now), 'CARS', '-v7.3')

end