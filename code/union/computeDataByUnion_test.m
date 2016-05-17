function computeDataByUnion_test()

%% Grid
grid_min = [-5; -5; -pi];
grid_max = [5; 5; pi];
N = [51; 51; 51];
pdDim = 3;
g = createGrid(grid_min, grid_max, N, pdDim);

%% time vector
dt = 0.025;
tIAT = 2;
tau = 0:dt:tIAT;

%% Problem parameters
schemeData.U = [-1 1];
schemeData.speed = 1;
schemeData.grid = g;
schemeData.hamFunc = @dubins3DHamFunc;
schemeData.partialFunc = @dubins3DPartialFunc;

%% Initial conditions
data0{1} = shapeCylinder(g, 3, [0; 0; 0], 0.5+0.5*rand);
data0{2} = shapeSphere(g, -1 + 2*rand(3,1), 1+rand);

%% Base reachable set
[g, base_data] = computeBaseBRS(g, tau, g.dx, schemeData);

for i = 1:length(data0)
  %% Compute reachable set directly
  dataTrue = HJIPDE_solve(data0{i}, tau, schemeData, 'zero');
  
  %% Compute reachable set by union
  tic
  dataUnion = computeDataByUnion(g, base_data(:,:,:,end), data0{i});
  toc
  
  %% Visualize
  figure
  hT = visSetIm(g, dataTrue(:,:,:,end));
  hT.FaceAlpha = 0.5;
  
  hold on  
  hU = visSetIm(g, dataUnion, 'b');
  hU.FaceAlpha = 0.5;
end

end