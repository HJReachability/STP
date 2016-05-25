function PE_BRS = compute_PE_BRS(Plane1, Plane2, PEmode, Rc, tau)
% [g, data] = compute_PE_BRS(Plane1, Plane2, Rc, tau)
%   Computes the backward reachable set for pursuit evasion
%
% Inputs:
%   Plane1 - this plane
%   Plane2 - other plane
%   PEmode - set to 'evade' if Plane1 is evader
%            set to 'pursue' if Plane1 is pursuer
%   Rc     - capture radius
%   tau    - time stamps
%
% Mo Chen, 2016-05-21

%% Default capture Planes, PEmode, capture radius, and time stamps
if nargin < 1
  Plane1 = Plane([0 0 0], 1, 5);
end

if nargin < 2
  Plane2 = Plane([0 0 0], 1, 5);
end

if nargin < 3
  PEmode = 'evade';
end

if nargin < 4
  Rc = 5;
end

if nargin < 5
  IAT = 3;
  dt = 0.1;
  tau = 0:dt:IAT;
end

%% Grid
grid_min = [-6; -10; 0];
grid_max = [20; 10; 2*pi];
N = [51; 51; 51];
pdDim = 3;
PE_BRS.g = createGrid(grid_min, grid_max, N, pdDim);

%% Copy over Plane parameters
schemeData.grid = PE_BRS.g;
schemeData.vrange1 = Plane1.vrange;
schemeData.vrange2 = Plane2.vrange;
schemeData.w1Max = Plane1.wMax;
schemeData.w2Max = Plane2.wMax;
schemeData.dMax = Plane1.dMax + Plane2.dMax;
schemeData.PEmode = PEmode;
schemeData.hamFunc = @dubins3DPEham;
schemeData.partialFunc = @dubins3DPEpartial;

%% Initial condition, solution, gradient, and output
data0 = shapeCylinder(g, 3, [0; 0; 0], Rc);
PE_BRS.data = HJIPDE_solve(data0, tau, schemeData, 'zero');

% Compute gradient
PE_BRS.grad = extractCostates(g, PE_BRS.data);


end