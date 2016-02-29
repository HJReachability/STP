function BRS = computeBRS(visualize)
% BRS = computeBRS(visualize)
% Computes the backwards reachable set from a target set
%
% Dynamics:
%   \dot{x} = v * cos(\theta) + d_1
%   \dot{y} = v * sin(\theta) + d_2
%   \dot{\theta} = u + d_3

if nargin<1
  visualize = true;
end

%---------------------------------------------------------------------------
% Resolution
res = [1; 1; 5*pi/180];

% Create the computation grid.
g.dim = 3;
g.min = [-20; -20; 0];
g.max = [+20; +20; 2*pi];
g.N = ceil((g.max - g.min) ./ res);
g.bdry = {@addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.max(3) = g.max(3) * (1 - 1 / g.N(3));

g = processGrid(g);

if prod(g.N) > 71^3
  disp(['g.N = ' num2str(g.N') '; continue?'])
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
% Problem parameters

% Target set is a cylinder in (x, y, \theta) space 
radius = 3;
target = shapeCylinder(g, 3, [0 0 0], radius);
v = 5; % constant speed
uMax = 1; % u \in [-uMax, uMax]
dMax = [1.5; 1.5; 0.3]; % d_i \in [-dMax(i), dMax(i)]

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @RASHamFunc;
schemeData.partialFunc = @RASPartialFunc;
schemeData.grid = g;
schemeData.d = dMax;

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

tau = t0;
reach = target;
data = target;

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
  y0 = data(:);
  
  % How far to step?
  tNext = min(tMax, tNow + tPlot);
  tSpan = [ tNow, tNext ];
  
  % Take a timestep.
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  % Get back the correctly shaped data array
  data = reshape(y, g.shape);
  data = min(data, target);
  tNow = t(end);
  
  reach = cat(4, reach, data);
  tau = cat(1, tau, -tNow);
  
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
end

BRS.g = g;
BRS.data = reach;
BRS.tau = tau;
BRS.v = v;
BRS.uMax = uMax;
BRS.dMax = dMax;

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = RASHamFunc(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.turnRate;
d = schemeData.d;

% Dynamics without disturbances
hamValue = v*deriv{1}.*cos(g.xs{3}) + v*deriv{2}.*sin(g.xs{3}) ...
  - w*abs(deriv{3});

% Add disturbances
hamValue = hamValue + abs(deriv{1})*d(1) + abs(deriv{2})*d(2) + ...
  abs(deriv{3})*d(3);

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

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.turnRate;
d = schemeData.d;

switch dim
  case 1
    alpha = v*abs(cos(g.xs{3})) + d(1);
    
  case 2
    alpha = v*abs(sin(g.xs{3})) + d(2);
    
  case 3
    alpha = w + d(3);
end
