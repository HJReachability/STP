function SPPwDisturbance_RTT(restart)

if nargin < 1
  restart = false;
end

% Computes reachable sets for getting to the target after intruder has passed
%% Grid
grid_min = [-1; -1; -3*pi/2]; % Lower corner of computation domain
grid_max = [1; 1; pi/2];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
schemeData.grid = createGrid(grid_min, grid_max, N, pdDims);

%% Time parameters
t0 = 0;
% Maximum time-horizon of computation (t = -tMax is the minimum absolute time)
tMax = 5;
% Set tau
dt = 0.01;
tau = t0:dt:tMax;

%% Problem parameters
% Vehicle
vrange = [0.1 1];
wMax = 1;
Rc = 0.1; % Capture radius
dMax = [0.1 0.2];

filename = sprintf('%s_checkpoint2.mat', mfilename);
numVeh = 4;
if restart
  Q = cell(numVeh,1);
  Q{1} = Plane([-0.1; 0; 0], wMax, vrange, dMax);
  Q{2} = Plane([ 0.1; 0; -pi], wMax, vrange, dMax);
  Q{3} = Plane([-0.1; 0.1; -pi/4], wMax, vrange, dMax);
  Q{4} = Plane([ 0.1; 0.1; -3*pi/4], wMax, vrange, dMax);
  
  %% target sets
  R = 0.1;
  Q{1}.data.target = shapeCylinder(schemeData.grid, 3, [0.7; 0.2; 0], R);
  Q{2}.data.target = shapeCylinder(schemeData.grid, 3, [-0.7; 0.2; 0], R);
  Q{3}.data.target = shapeCylinder(schemeData.grid, 3, [0.7; -0.7; 0], R);
  Q{4}.data.target = shapeCylinder(schemeData.grid, 3, [-0.7; -0.7; 0], R);
  
  %% Reduced target set for the first BRS
  Rsmall = 0.025;
  Q{1}.data.targetsm = shapeCylinder(schemeData.grid, 3, [0.7; 0.2; 0], Rsmall);
  Q{2}.data.targetsm = shapeCylinder(schemeData.grid, 3, [-0.7; 0.2; 0], Rsmall);
  Q{3}.data.targetsm = shapeCylinder(schemeData.grid, 3, [0.7; -0.7; 0], Rsmall);
  Q{4}.data.targetsm = shapeCylinder(schemeData.grid, 3, [-0.7; -0.7; 0], Rsmall);
else
  load(filename)
  Q = {Q1; Q2; Q3; Q4};
end

%% Base obstacle generation method
baseObs_method = 'RTT';
if strcmp(baseObs_method, 'RTT')
  fprintf('Using %s method to generate base obstacles\n', baseObs_method)
  load('RTTRS.mat')
  baseObs_params.RTTRS = ...
    migrateGrid(RTTRS.g, -RTTRS.data(:,:,:,end), schemeData.grid);
  figure;
  h1 = visSetIm(RTTRS.g, -RTTRS.data(:,:,:,end));
  h1.FaceAlpha = 0.5;
  hold on
  
  h3 = visSetIm(schemeData.grid, baseObs_params.RTTRS);
  h3.FaceAlpha = 0.5;
  h3.FaceColor = 'b';
  %% Reduced speed if using robust tracker
  for i = 1:4
    Q{i}.data.vReserved = [0.3 -0.3];
    Q{i}.data.wReserved = -0.4;
    Q{i}.data.RTT_radius = 0.075;
  end
  
elseif strcmp(baseObs_method, 'CC')
  %% Reset radius for base obstacle computation
  baseObs_params.resetR = [0.03, 0.03, 0.1]';
  
end

%% Start the computation of reachable sets
for veh=1:numVeh
  schemeData.dynSys = Q{veh};
  
  %% Gather induced obstacles of higher-priority vehicles
  % Assume there's no static obstacle
  obstacles = gatherCylObs3D(Q(1:veh-1), schemeData, tau);
  
  %% Compute the BRS (BRS1) of the vehicle with the above obstacles
  if ~isfield(Q{veh}.data, 'BRS1')
    fprintf('Computing BRS1 for vehicle %d\n', veh)
    Q{veh} = computeBRS1(Q{veh}, tau, schemeData, obstacles);
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Compute the base obstacles for based on BRS1
  if ~isfield(Q{veh}.data, 'baseObs')
    if veh < numVeh
      fprintf('Computing base obstacles for vehicle %d\n', veh)
      Q{veh} = ...
        computeBaseObs(Q{veh}, schemeData, baseObs_method, baseObs_params);
    end
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
  
  %% Flatten obstacles to 3D, add capture radius, and unflatten to 3D
  if ~isfield(Q{veh}.data, 'flatObs2D')
    if veh < numVeh
      fprintf('Flattening obstacles for vehicle %d\n', veh)
      Q{veh} = flatAugBaseObs(Q{veh}, schemeData, Rc);
    end
    
    [Q1, Q2, Q3, Q4] = Q{:};
    save(filename, 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', '-v7.3')
  end
end
end