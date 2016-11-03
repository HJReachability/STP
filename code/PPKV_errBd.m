function PPKV_errBd(tR, save_png)

if nargin < 2
  save_png = true;
end

%% Parameters
% Plane parameters
vRangeA = [0.25 2.5];
wMaxA = 2;
dMaxA = 0.2*[max(vRangeA) wMaxA];

% Kinematic vehicle parameters
vMax = 2;

% Pursuit evasion game
PPKV = PlanePursue2DKV([0;0;0], wMaxA, vRangeA, dMaxA, vMax);
schemeData.dynSys = PPKV;


%% Compute reachable set
% Grid
grid_min = [-1.25*tR; -1.25*tR; -pi]; % Lower corner of computation domain
grid_max = [1.25*tR; 1.25*tR; pi];    % Upper corner of computation domain
% Number of grid points per dimension
% N = [101; 101; 101]; % for SPPwIntruderRTT method 1
N = [71; 71; 95]; % for SPPwIntruderRTT method 2
g = createGrid(grid_min, grid_max, N, 3);
schemeData.grid = g;

% Track trajectory for up to this time
% tMax = 2; % for SPPwIntruderRTT method 1
tMax = 10; % for SPPwIntruderRTT method 2
dt = 0.1;
tau = 0:dt:tMax;

% Initial conditions
data0 = -shapeCylinder(schemeData.grid, 3, [ 0; 0; 0 ], tR);
schemeData.uMode = 'max';
schemeData.dMode = 'min';
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.stopInit = [0;0;0];

if save_png
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
  extraArgs.fig_filename = sprintf('%s/', folder);
end

% Compute
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

save(sprintf('%s/data.mat', folder), g, data)
end