function computeEvade()
addpath(genpath('engine'))
grid_min = [-5; -7.5; -pi];
grid_max = [10; 7.5; pi];
N = [51; 51; 51];
pdDim = 3;
g = createGrid(grid_min, grid_max, N, pdDim);

schemeData.grid = g;
schemeData.v1 = 1;
schemeData.vI = 1;
schemeData.u1Max = 1;
schemeData.uIMax = 1;
schemeData.d1Max = [0.1; 0.1; 0.1];
schemeData.dIMax = [0.1; 0.1; 0.1];
schemeData.hamFunc = @evadeHam;
schemeData.partialFunc = @evadePartial;

Rc = 4;
data0 = shapeCylinder(g, 3, [0; 0; 0], Rc);

IAT = 2;
dt = 0.05;
tau = 0:dt:IAT;
data = HJIPDE_solve(data0, tau, schemeData, 'zero');

save('evadeVF.mat', 'g', 'data')

visSetIm(g, data(:,:,:,end));
end