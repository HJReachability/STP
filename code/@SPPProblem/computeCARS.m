function computeCARS(obj, Qintr, save_png, restart)
% computeCARS(obj, Rc, tIAT, save_png)
%     Computes collision avoidance reachable set and updates the SPPP object
%     with the CARS file name
%
% Inputs:
%     obj - SPP problem object
%     Qintr - intruder vehicle object (just for extracting vehicle parameters)
%     tIAT - intruder avoidance time
%     save_png - set to true to save computation figures

% By default, intruder plane has the same control and disturbance bounds as SPP
% vehicles
if nargin < 2
  Qintr = Plane([0; 0; 0], obj.wMaxA, obj.vRangeA, obj.dMaxA);
end

if nargin < 4
  save_png = true;
end

if nargin < 5
  restart = false;
end

if ~restart
  if exist(obj.CARS_filename, 'file')
    fprintf('The CARS file %s already exists. Skipping CARS computation.\n', ...
      obj.CARS_filename)
    return
  end
end

%% Problem parameters
schemeData.dynSys = ...
  PlaneCAvoid([0; 0; 0], obj.wMaxA, obj.vRangeA, Qintr.wMax, Qintr.vrange, ...
  obj.dMaxA, Qintr.dMax);
schemeData.uMode = 'max';
schemeData.dMode = 'min';

%% Grid and target set
% % for SPPwIntruderRTT method 1
% grid_min = [-3*obj.Rc; -4*obj.Rc; 0];
% grid_max = [5*obj.Rc; 4*obj.Rc; 2*pi];
% N = [101; 101; 101]; 

% % for SPPwIntruderRTT method 2 with 11 m/s wind
% grid_min = [-27*obj.Rc; -27*obj.Rc; 0];
% grid_max = [27*obj.Rc; 27*obj.Rc; 2*pi];

% % for SPPwIntruderRTT method 2 with 6 m/s wind
% grid_min = [-20; -20; 0];
% grid_max = [20; 20; 2*pi];
% 
% % For 0 m/s wind
% grid_min = [-4; -4; 0];
% grid_max = [4; 4; 2*pi];
% 
% N = [41; 41; 41];

% For toy example
grid_min = [-25; -25; 0];
grid_max = [25; 25; 2*pi];
N = [251; 251; 251];

% 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, 3);

data0 = shapeCylinder(schemeData.grid, 3, [0; 0; 0], obj.Rc);

%% Time stamps
tau = 0:obj.dt:obj.tIAT;

%% Compute set
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

if save_png
  if ispc
    folder = sprintf('%s\\%s', obj.folder, mfilename);
    system(sprintf('mkdir %s', folder));
  else
    folder = sprintf('%s/%s', obj.folder, mfilename);
    system(sprintf('mkdir -p %s', folder));
  end
  
  extraArgs.fig_filename = sprintf('%s/', folder);
end

data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

CARS.dynSys = schemeData.dynSys;
CARS.g = schemeData.grid;
CARS.data = data;
CARS.tau = tau;

% Update SPPP and save
obj.CARS_filename = sprintf('%s/CARS.mat', obj.folder);
save(obj.CARS_filename, 'CARS', '-v7.3')

obj.buffer_duration = max(CARS.tau)/obj.max_num_affected_vehicles;
obj.buffer_duration_ind = find(CARS.tau >= obj.buffer_duration, 1, 'first');
obj.remaining_duration_ind = length(CARS.tau) - obj.buffer_duration_ind;
SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end