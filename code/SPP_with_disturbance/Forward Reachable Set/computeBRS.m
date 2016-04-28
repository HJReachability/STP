function BRS = computeBRS(tradius, speed, uMax, dMax, res, visualize, endEarly)
% BRS = computeBRS(visualize)
% Computes the backwards reachable set from a target set
%
% Dynamics:
%   \dot{x} = v * cos(\theta) + d_1
%   \dot{y} = v * sin(\theta) + d_2
%   \dot{\theta} = u + d_3

%% Problem parameters
if nargin < 1
  tradius = 0.1;
end

if nargin < 2
  speed = 1; % constant speed
end
v = speed;

if nargin < 3
  uMax = 1; % u \in [-uMax, uMax]
end

if nargin < 4
  dMax = [0.1; 0.2]; % [radius in (x,y) space; bounds in theta space]
end

% Resolution
if nargin < 5
  res = [0.075; 0.075; 5*pi/180];
end

if nargin < 6
  visualize = true;
end

if nargin < 7
  endEarly = false;
end

%% Create the computation grid.
g.dim = 3;
g.min = [-2; -2; 0];
g.max = [+2; +2; 2*pi];
g.N = ceil((g.max - g.min) ./ res);
g.bdry = {@addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.max(3) = g.max(3) * (1 - 1 / g.N(3));

g = processGrid(g);

if prod(g.N) > 71^3
  warning(['g.N = ' num2str(g.N')])
end

data = shapeCylinder(g, 3, [0 0 0], tradius);

RF = 1.25; % Refinement factor

if nnz(data(:) <= 0) <= 5 * g.N(3)
  disp('Target set is too small for this grid resolution!')
  disp(['Changing resolution from [' num2str(res(1)) '; ' num2str(res(2)) ...
    '; ' num2str(res(3)) '] to [' num2str(res(1)/RF) '; ' ...
    num2str(res(2)/RF) '; ' num2str(res(3)) ']'])
  
  res(1:2) = res(1:2) / RF;
  BRS = computeBRS(tradius, speed, uMax, dMax, res, visualize, true);
  data = migrateGrid(BRS.g, BRS.data, g);
end

%---------------------------------------------------------------------------
% Integration parameters.
plotSteps = 12;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
tMax = 5;                  % End time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% Pause after each plot?
pauseAfterPlot = 0;

% Plot in separate subplots (set deleteLastPlot = 0 in this case)?
useSubplots = 0;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
accuracy = 'veryHigh';

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @RASHamFunc;
schemeData.partialFunc = @RASPartialFunc;
schemeData.grid = g;
schemeData.dMax = dMax;

% The Hamiltonian and partial functions need problem parameters.
schemeData.velocity = v;
schemeData.turnRate = uMax;
%--------------------------------------------------------------------------
% Choose degree of dissipation.

switch(dissType)
  case 'global'
    schemeData.dissFunc = @artificialDissipationGLF;
  case 'local'
    schemeData.dissFunc = @artificialDissipationLLF;
  case 'locallocal'
    schemeData.dissFunc = @artificialDissipationLLLF;
  otherwise
    error('Unknown dissipation function %s', dissFunc);
end

%--------------------------------------------------------------------------
% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.5, 'stats', 'on');

% Choose approximations at appropriate level of accuracy.
switch(accuracy)
  case 'low'
    schemeData.derivFunc = @upwindFirstFirst;
    integratorFunc = @odeCFL1;
  case 'medium'
    schemeData.derivFunc = @upwindFirstENO2;
    integratorFunc = @odeCFL2;
  case 'high'
    schemeData.derivFunc = @upwindFirstENO3;
    integratorFunc = @odeCFL3;
  case 'veryHigh'
    schemeData.derivFunc = @upwindFirstWENO5;
    integratorFunc = @odeCFL3;
  otherwise
    error('Unknown accuracy level %s', accuracy);
end

if(singleStep)
  integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
end

if visualize
  %---------------------------------------------------------------------------
  % Initialize Display
  figure;
  
  % Set up subplot parameters if necessary.
  if(useSubplots)
    cols = ceil(sqrt(plotSteps));
    rows = ceil(plotSteps / cols);
    plotNum = 1;
    subplot(rows, cols, plotNum);
  end
  
  h = visualizeLevelSet(g, data, 'surface', 0);
  camlight right
  camlight left
  axis(g.axis);
  axis square
  drawnow;
end
%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
while(tMax - tNow > small * tMax)
  % Reshape data array into column vector for ode solver call.
  data_last = data;
  y0 = data(:);
  
  % How far to step?
  tNext = min(tMax, tNow + tPlot);
  tSpan = [ tNow, tNext ];
  
  % Take a timestep.
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  % Get back the correctly shaped data array
  data = reshape(y, g.shape);
  data = min(data, data_last);
  tNow = t(end);
  
  if visualize
    if(pauseAfterPlot)
      % Wait for last plot to be digested.
      pause;
    end
    
    delete(h);
    h = visualizeLevelSet(g, data, 'surface', 0);
    axis square
    drawnow;
  end
  
  if endEarly
    if nnz(data(:) <= 0) > 40 * g.N(3)
      disp(['Reverting resolution back to [' num2str(res(1)*RF) '; ' ...
        num2str(res(2)*RF) '; ' num2str(res(3)) ']'])
      break;
    end
  end
end

BRS.g = g;
BRS.data = data;
BRS.tMax = tNow;
BRS.v = v;
BRS.uMax = uMax;
BRS.dMax = dMax;
BRS.P = extractCostates(g, data);

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = RASHamFunc(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'velocity', 'turnRate', 'dMax');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.turnRate;
dMax = schemeData.dMax;

% Dynamics without disturbances
hamValue = v*deriv{1}.*cos(g.xs{3}) + v*deriv{2}.*sin(g.xs{3}) ...
  - w*abs(deriv{3});

% Add disturbances
hamValue = hamValue + dMax(1) * sqrt(deriv{1}.^2 + deriv{2}.^2) + ...
  abs(deriv{3}) * dMax(2);

hamValue = -hamValue;

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = RASPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% air3DPartialFunc: Hamiltonian partial fcn for 3D collision avoidance example.
%
% alpha = air3DPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
%
% This function implements the partialFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the extrema of the absolute value of the partials of the
%   analytic Hamiltonian with respect to the costate (gradient).
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   derivMin	 Cell vector of minimum values of the costate (\grad \phi).
%   derivMax	 Cell vector of maximum values of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%   dim          Dimension in which the partial derivatives is taken.
%
%   alpha	 Maximum absolute value of the partial of the Hamiltonian
%		   with respect to the costate in dimension dim for the
%                  specified range of costate values (O&F equation 5.12).
%		   Note that alpha can (and should) be evaluated separately
%		   at each node of the grid.
%
%
% Ian Mitchell 3/26/04

checkStructureFields(schemeData, 'grid', 'velocity', 'turnRate', 'dMax');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.turnRate;
dMax = schemeData.dMax;

switch dim
  case 1
    alpha = v*abs(cos(g.xs{3})) + ...
      dMax(1) * abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    
  case 2
    alpha = v*abs(sin(g.xs{3})) + ...
      dMax(1) * abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    
  case 3
    alpha = w + dMax(2);
end
