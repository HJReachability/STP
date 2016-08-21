function computeCARS(obj, Qintr, Rc, tIAT, save_png)
% computeCARS(obj, Rc, tIAT, save_png)
%     Computes collision avoidance reachable set and updates the SPPP object
%     with the CARS file name
%
% Inputs:
%     obj - SPP problem object
%     Qintr - intruder vehicle object (just for extracting vehicle parameters)
%     Rc - capture radius
%     tIAT - intruder avoidance time
%     save_png - set to true to save computation figures

% By default, intruder plane has the same control and disturbance bounds as SPP
% vehicles
if nargin < 2
  Qintr = Plane([0; 0; 0], obj.wMaxA, obj.vRangeA, obj.dMaxA);
end

if nargin < 3
  Rc = 0.1;
end

if nargin < 4
  tIAT = 0.25;
end

if nargin < 5
  save_png = true;
end

if exist(obj.CARS_filename, 'file')
  fprintf('The CARS file %s already exists. Skipping CARS computation.\n', ...
    obj.CARS_filename)
  return
end

%% Problem parameters
schemeData.dynSys = ...
  PlaneCAvoid([0; 0; 0], obj.wMaxA, obj.vRangeA, Qintr.wMax, Qintr.vrange, ...
  obj.dMaxA, Qintr.dMax);
schemeData.uMode = 'max';
schemeData.dMode = 'min';

%% Grid and target set
grid_min = [-0.3; -0.4; 0]; % Lower corner of computation domain
grid_max = [0.5; 0.4; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

data0 = shapeCylinder(schemeData.grid, 3, [0; 0; 0], Rc);

%% Time stamps
tau = 0:obj.dt:tIAT;

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
CARS.Rc = Rc;

% Update SPPP and save
obj.CARS_filename = sprintf('CARS_%f.mat', now);
save(obj.CARS_filename, 'CARS', '-v7.3')
end