function computeRTTRS(obj, save_png)
% computeRTTRS(obj, vR, wR, trackingRadius, save_png)
%     Computes the robust trajectory tracking reachable set and updates the SPPP
%     object with the RTTRS file name
%
% Inputs:
%     obj - SPP problem object
%     vR - reserved vehicle speed
%     wR - reserved angular acceleration
%     tR - tracking radius
%     save_png - set to true to save figures
%
% Output:
%     SPPP - SPP problem object updated with RTTRS file name

% if nargin < 2
%   vR = [0.3 -0.3];
% %   vR = [0.25 -0.25];  
% end
% 
% if nargin < 3
%   wR = -0.4;
% end
% 
% if nargin < 4
%   tR = 0.075;
% end

if nargin < 2
  save_png = true;
end

if exist(obj.RTTRS_filename, 'file')
  fprintf('The RTTRS file %s already exists. Skipping RTTRS computation.\n', ...
    obj.RTTRS_filename)
  return
end

vR = obj.vReserved;
wR = obj.wReserved;
tR = obj.RTT_tR;

% Grid
grid_min = [-1.25*tR; -1.25*tR; -pi]; % Lower corner of computation domain
grid_max = [1.25*tR; 1.25*tR; pi];    % Upper corner of computation domain
% Number of grid points per dimension
% N = [101; 101; 101]; % for SPPwIntruderRTT method 1
N = [51; 51; 101]; % for SPPwIntruderRTT method 2
schemeData.grid = createGrid(grid_min, grid_max, N, 3);

% Track trajectory for up to this time
% tMax = 2; % for SPPwIntruderRTT method 1
tMax = 30; % for SPPwIntruderRTT method 2
dt = 0.1;
tau = 0:dt:tMax;

% Virtual vehicle to be tracked
vRangeB = obj.vRangeA + vR;
wMaxB = obj.wMaxA + wR;
dMaxB = [0 0];
dynSys = PlaneCAvoid(zeros(3,1), obj.wMaxA, obj.vRangeA, wMaxB, vRangeB, ...
  obj.dMaxA, dMaxB);
schemeData.dynSys = dynSys;

% Initial conditions
RTTRS.trackingRadius = tR;
data0 = -shapeCylinder(schemeData.grid, 3, [ 0; 0; 0 ], RTTRS.trackingRadius);
schemeData.uMode = 'max';
schemeData.dMode = 'min';
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.stopInit = [0;0;0];

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

% Compute
data = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);

RTTRS.g = schemeData.grid;
RTTRS.data = data(:,:,:,end);
RTTRS.dynSys = dynSys;

% Save results
obj.RTTRS_filename = sprintf('%s/RTTRS.mat', obj.folder);
save(obj.RTTRS_filename, 'RTTRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end