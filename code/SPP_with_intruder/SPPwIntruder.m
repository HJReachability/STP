%% To-dos
% - Add a for loop with veh as index
% - Load the base BRS and FRS datasets appropriately and change the name of
%   variables so that they are consistent with this file.
% - Use the stopSet functionality correctly in Step-3b
% - Use the new addCRadius function appropriately
% - Test the base obstacle computation function
% - Write a function for the final simulation

% This function initializes the simulation for solving the SPP problem in
% the presence of intruder. 

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

% Base grid - it will be used for computation of recahable sets by union 
Nfine = [151; 151; 151];
base_g = createGrid(grid_min, grid_max, Nfine, pdDims);

%% time vector
t0 = 0;
tMax = 4;
dt = 0.01;
tIAT = 0.2;

%% problem parameters
% Vehicle
speed = [0.5 1];
U = 1;
Rc = 0.1;
dMax = [0.1 0.2];

% Intruder
speedI = [0.25 0.75]; 
UI = 0.5;

%% Pack problem parameters
schemeData.grid = g; % Grid MUST be specified!
schemeData.wMax = U;
schemeData.vrange = speed;
schemeData.dMax = dMax;
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';
schemeData.accuracy = 'medium';

% System dynamics
schemeData.hamFunc = @dubins3Dham;
schemeData.partialFunc = @dubins3Dpartial;

%% target set
R = 0.1;
Q{1}.data0 = shapeCylinder(g, 3, [0.7; 0.2; 0], R);
Q{2}.data0 = shapeCylinder(g, 3, [-0.7; 0.2; 0], R);
Q{3}.data0 = shapeCylinder(g, 3, [0.7; -0.7; 0], R);
Q{4}.data0 = shapeCylinder(g, 3, [-0.7; -0.7; 0], R);

%% initial States
Q{1}.initState = [-0.5, 0, 0]';
Q{2}.initState = [ 0.5, 0, pi]';
Q{3}.initState = [-0.6, 0.6, 7*pi/4]';
Q{4}.initState = [ 0.6, 0.6, 5*pi/4]';

%% Compute the base FRS and base BRS
Need to fill in
filename = ['baseBRS_' num2str(schemeData.wMax) ...
  '_' num2str(max(schemeData.vrange)) '.mat'];

if exist(filename, 'file')
  load(filename)
else
  tau = t0:dt:tIAT;
  schemeDataFine = schemeData;
  schemeData.uMode = 'max';
  schemeData.dMode = 'max';
  schemeData.tmode = 'forward';
  schemeDataFine.grid = base_g;
  base_data0 = shapeRectangleByCorners(base_g, -g.dx/2, g.dx/2);
  extraArgs.visualize = true;
  extraArgs.plotData.plotDims = [1, 1, 0];
  extraArgs.plotData.projpt = [0];
  
  base_data = HJIPDE_solve(base_data0, tau, schemeDataFine, ...
    'zero', extraArgs);
  save(filename, 'base_g', 'base_data', '-v7.3')
end

%% Reset extraArgs
extraArgs = [];

%% Step-1: Find out the set of obstacles induced by the higher priority
% vehicles. It should be simply the base obstacles augmented by a tIAT-step
% FRS and then union over all vehicles. Since the computations are done
% recursively, we will only compute the obstacles for the last vehicle.

if veh ~= 1
  
  % Extract the base obstacles
  obstacles = Q{veh-1}.Obs;
  obstau = Q{veh-1}.Obstau;
  numObs = size(obstacles, g.dim+1);
  
  % Append the base obstacles by a tIAT step FRS
  % Index of the base FRS
  base_indFRS = find(base_tauFRS == tIAT);
  for i= 1:numObs
    obstacles(:, :, :, i) = computeDataByUnion(base_gFRS, ...
      base_dataFRS(:, :, :, base_indFRS), g, obstacles(:, :, :, i));
    % Subtract the part of the obstacle that hits the target
    obstacles(:, :, :, i) = shapeDifference(obstacles(:, :, :, i), ...
      Q{veh-1}.data0);
  end
  
  % Shift the obstacles sequence forward by tIAT to get the correct
  % obstacle sequence
  shifts = floor((tIAT - t0)/dt);
  % For the tIAT time just use the first obstacle 
  obstacles(:, :, :, 1:shifts) = repmat(obstacles(:, :, :, 1), ...
    [ones(1,g.dim) shifts]);
  % Thereafter shift the obstacles by tIAT 
  obstacles(:, :, :, shifts+1:end) = obstacles(:, :, :, 1:end-shifts);
  
  % Add capture radius to obstacles
  for i= 1:numObs
    [g2D, data2D] = proj2D(g, obstacles(:, :, :, i), [0,0,1]);
    data2D = addCRadius(g2D, data2D, Rc);
    obstacles(:, :, :, i) = repmat(data2D, [1,1,g.shape(3)]);
  end
  
  % Assign this obstacle sequence to the vehicle
  Q{veh-1}.Obs = obstacles;
  
  % Finally, take union of obstacles across all vehicles to get the overall
  % obstacle set
  if exist('unionObs', 'var')
    for i=1:numObs
      unionObs(:, :, :, i) = shapeUnion(unionObs(:, :, :, i), ...
        obstacles(:, :, :, i));
    end
  else
    unionObs = obstacles;
  end
  
end 
  

%% Step-2a: Augment the obstacles by a tIAT step BRS
% It should be simply the obstacles in Step1 augmented by a tIAT-step
% BRT and then union over all time steos for the duraction [0 tIAT]. 

if veh ~= 1
  
  % Append the obstacle i-step BRS of the obstacle corresponding to time
  % t0-i, where t0 is the current time.
  obstacles = []; % Reset the obstacle matrix
  
  for i = 1:numObs
    obstacles(:, :, :, i) = unionObs(:, :, :, i);
    j = i + 1; % Index of the obstacle
    t = dt; % t tracks time
    
    while (j >= 1 && j <= numObs && t <= tIAT)
      base_indBRS = find(base_tauBRS == t);
      obstoAdd = computeDataByUnion(base_gBRS, ...
        base_dataBRS(:, :, :, base_indBRS), g, unionObs(:, :, :, j));
      obstacles(:, :, :, i) = shapeUnion(obstacles(:, :, :, i), obstoAdd);
      j = j + 1;
      t = t + dt;
    end
  end
  
  % Set the extraArgs obstacle field
  extraArgs.obstacles = obstacles;
end 

% Save the sets, just in case
filename = sprintf('SPPwIntruder_check1_%f', veh);
save(filename, 'Q', 'unionObs', '-v7.3')


%% Step-2b: Compute the BRS of the vehicle with the above obstacles

% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% Set tau
tau = 0:dt:tMax;

% Set extraArgs
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];

% Computation should stop once it contains the initial state
extraArgs.stopInit.initState = Q{veh}.initState;

[data, tau, ~] = HJIPDE_solve(Q{veh}.data0, tau, schemeData,...
  'zero', extraArgs);  

% Assign these sets to the vehicle 
Q{veh}.data_BRS1 = data;
Q{veh}.tau_BRS1 = tau;

%% Step-3a: Augment the BRS in Step-2 by a tIAT step FRS

% Set schemeData
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% Set extraArgs
extraArgs = [];
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];

% Set tau
tau = 0:dt:tIAT;

dataFRS = HJIPDE_solve(data(:,:,:,end), tau, schemeData,...
  'zero', extraArgs);

%% Step-3b: Compute a BRS that contains the FRS in Step-3a

% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'max';
schemeData.tMode = 'backward';

% Set tau
tau = 0:dt:tMax;

% Set extraArgs
extraArgs = [];
extraArgs.visualize = true;
extraArgs.plotData.plotDims = [1, 1, 0];
extraArgs.plotData.projpt = [0];

% Stop the computation once the BRS includes the FRS (and thus also
% contains the initial state)
extraArgs.stopSet = dataFRS(:,:,:,end);

if veh ~= 1
  extraArgs.obstacles = unionObs;
end

[data, tau, ~] = HJIPDE_solve(Q{veh}.data0, tau, schemeData,...
  'zero', extraArgs);

% Assign these sets to the vehicle 
Q{veh}.data_BRS2 = data;
Q{veh}.tau_BRS2 = tau;

% Save the sets, just in case
filename = sprintf('SPPwIntruder_check2_%f', veh);
save(filename, 'Q', 'unionObs', '-v7.3')

%% Step-3c: Compute the base obstacles for vehicles

% Set schemeData
schemeDataBaseObs = schemeData;
schemeDataBaseObs.uMode = 'max';
schemeDataBaseObs.dMode = 'max';
schemeDataBaseObs.tMode = 'forward';

% System dynamics
schemeDataBaseObs.hamFunc = @dubins3Dham_baseObs;
schemeDataBaseObs.partialFunc = @dubins3Dpartial_baseObs;



%% Step-3d: Fix the time-scale of the sets