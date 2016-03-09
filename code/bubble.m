function [data, g, data0] = bubble(radius)
% air3D: demonstrate the 3D aircraft collision avoidance example
%
% [data, g, data0] = bubble(radius)
%
% The relative coordinate dynamics are
%
%   \dot x    = -v_a + v_b \cos \psi + a y
%	  \dot y    = v_b \sin \psi - a x
%	  \dot \psi = b - a
%
% where v_a and v_b are constants, input a is trying to avoid the target
%	input b is trying to hit the target.
%
% For more details, see my PhD thesis, section 3.1.
%
% Output Parameters:
%
%   data: Implicit surface function at t_max.
%
%   g: Grid structure on which data was computed.
%
%   data0: Implicit surface function at t_0.

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 4;                  % End time.
plotSteps = 50;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% Problem Parameters.
%   targetRadius  Radius of target circle (positive).
%   velocityA	  Speed of the evader (positive constant).
%   velocityB	  Speed of the pursuer (positive constant).
%   inputA	  Maximum turn rate of the evader (positive).
%   inputB	  Maximum turn rate of the pursuer (positive).
if nargin < 1
  radius = 0.1;
end

vNom = 0.75;
vRange = [0.5 1];
wNom = 0.6;
wMax = 1;
dMax = [0.1; 0.2]; % [radius in (x,y) space; bounds in theta space]

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Visualize the 3D reachable set.
displayType = 'surface';

% Delete previous plot before showing next?
deleteLastPlot = 1;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 51;

% Create the grid.
g.dim = 3;
g.min = [-1.2*radius; -1.2*radius; -pi/3];
g.max = [ 1.2*radius;  1.2*radius;  pi/3];
g.bdry = {@addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; ceil(Nx * (g.max(2) - g.min(2)) / (g.max(1) - g.min(1))); Nx ];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
% g.max(3) = g.max(3) * (1 - 1 / g.N(3));
g = processGrid(g);

%---------------------------------------------------------------------------
% Create initial conditions (cylinder centered on origin).
data = -shapeCylinder(g, 3, [ 0; 0; 0 ], radius);
data0 = data;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @air3DHamFunc;
schemeData.partialFunc = @air3DPartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.vNom = vNom;
schemeData.vRange = vRange;
schemeData.wNom = wNom;
schemeData.wMax = wMax;
schemeData.dMax = dMax;

%---------------------------------------------------------------------------
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

accuracy = 'veryHigh';

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.75, 'stats', 'on');

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

%---------------------------------------------------------------------------
% Restrict the Hamiltonian so that reachable set only grows.
%   The Lax-Friedrichs approximation scheme MUST already be completely set up.
innerFunc = schemeFunc;
innerData = schemeData;
clear schemeFunc schemeData;

% Wrap the true Hamiltonian inside the term approximation restriction routine.
schemeFunc = @termRestrictUpdate;
schemeData.innerFunc = innerFunc;
schemeData.innerData = innerData;
schemeData.positive = 0;

%---------------------------------------------------------------------------
% Initialize Display
f = figure;

h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);

camlight right;
camlight left;
hold on;
axis(g.axis);
drawnow;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
while(tMax - tNow > small * tMax)

  % Reshape data array into column vector for ode solver call.
  y0 = data(:);

  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
  % Take a timestep.
  [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
                  integratorOptions, schemeData);
  tNow = t(end);

  % Get back the correctly shaped data array
  data = reshape(y, g.shape);

  % Get correct figure, and remember its current view.
  figure(f);

  % Delete last visualization if necessary.
  if(deleteLastPlot)
    delete(h);
  end

  % Create new visualization.
  h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);  
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
end

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = air3DHamFunc(t, data, deriv, schemeData)
% air3DHamFunc: analytic Hamiltonian for 3D collision avoidance example.
%
% hamValue = air3DHamFunc(t, data, deriv, schemeData)
%
% This function implements the hamFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the analytic Hamiltonian for such a flow field.
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   deriv	 Cell vector of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%
%   hamValue	 The analytic hamiltonian.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.
%   .velocityA	 Speed of the evader (positive constant).
%   .velocityB	 Speed of the pursuer (positive constant).
%   .inputA	 Maximum turn rate of the evader (positive).
%   .inputB	 Maximum turn rate of the pursuer (positive).
%
% Ian Mitchell 3/26/04

checkStructureFields(schemeData, 'grid', 'vNom', 'vRange', ...
                                 'wNom', 'wMax', 'dMax');

grid = schemeData.grid;
vNom = schemeData.vNom;
vRange = schemeData.vRange;
wNom = schemeData.wNom;
wMax = schemeData.wMax;
dMax = schemeData.dMax;

% Dynamics:
%   \dot x    = -v_a + v_b \cos \psi + a y + d_1
%	  \dot y    = v_b \sin \psi - a x + d_2
%	  \dot \psi = b - a + d_3
% b is control of actual vehicle: drives state away from target -> maximize
% a is virtual vehicle: drives state into target -> minimize
% d is disturbance: drives state into target -> minimize

hamValue1 = deriv{1} .* (-vNom) + ... % p_1 * -v_a
  (deriv{1}.*cos(grid.xs{3}) >= 0) .* ... % p_1 * cos x_3 * v_b
    (deriv{1}.*cos(grid.xs{3})) * vRange(2) + ...
  (deriv{1}.*cos(grid.xs{3}) < 0) .* ...
    (deriv{1}.*cos(grid.xs{3})) * vRange(1) + ... % p_1 * x_2 * a
  (deriv{1}.*grid.xs{2} >= 0) .* (deriv{1}.*grid.xs{2}) * (-wNom) + ...
  (deriv{1}.*grid.xs{2} < 0) .* (deriv{1}.*grid.xs{2}) * (wNom);

hamValue2 = (deriv{2}.*sin(grid.xs{3}) >= 0) .* ... % p_2 * sin x_3 * v_b
  (deriv{2}.*sin(grid.xs{3})) * vRange(2) + ...
  (deriv{2}.*sin(grid.xs{3}) < 0) .* ...
  (deriv{2}.*sin(grid.xs{3})) * vRange(1) + ...                % p_2 * -x_1 * a
  (deriv{2}.*(-grid.xs{1}) >= 0) .* (deriv{2}.*(-grid.xs{1})) * (-wNom) + ...
  (deriv{2}.*(-grid.xs{1}) < 0) .* (deriv{2}.*(-grid.xs{1})) * (wNom);

hamValue3 = (deriv{3} >= 0) .* deriv{3} * (wMax + (-wNom) + (-dMax(2))) + ...
  (deriv{3} < 0) .* deriv{3} * (-wMax + wNom + dMax(2));
  
% Dealing with disturbance in the first two components
% (d_x, d_y) points in the opposite direction of (p_1, p_2) --> -d(1)*norm(p)
hamValued12 = -dMax(1) * sqrt(deriv{1}.^2 + deriv{2}.^2);

% Backwards reachable set
hamValue = -(hamValue1 + hamValue2 + hamValue3 + hamValued12);
end


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = air3DPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
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
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):
%
%   .grid	 Grid structure.
%   .velocityA	 Speed of the evader (positive constant).
%   .velocityB	 Speed of the pursuer (positive constant).
%   .inputA	 Maximum turn rate of the evader (positive).
%   .inputB	 Maximum turn rate of the pursuer (positive).
%
% Ian Mitchell 3/26/04

checkStructureFields(schemeData, 'grid', 'vNom', 'vRange', ...
                                 'wNom', 'wMax', 'dMax');

grid = schemeData.grid;
vNom = schemeData.vNom;
vRange = schemeData.vRange;
wNom = schemeData.wNom;
wMax = schemeData.wMax;
dMax = schemeData.dMax;

% Dynamics:
%   \dot x    = -v_a + v_b \cos \psi + a y + d_1
%	  \dot y    = v_b \sin \psi - a x + d_2
%	  \dot \psi = b - a + d_3
% b is control of actual vehicle: drives state away from target -> maximize
% a is virtual vehicle: drives state into target -> minimize
% d is disturbance: drives state into target -> minimize

switch dim
  case 1
    alpha = abs(-vNom + vRange(2) * cos(grid.xs{3})) + ...
      wNom * abs(grid.xs{2}) + ...
      dMax(1) * abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);

  case 2
    alpha = abs(vRange(2) * sin(grid.xs{3})) + wNom * abs(grid.xs{1}) + ... 
      dMax(1) * abs(derivMax{2}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);

  case 3
    alpha = wNom + wMax + dMax(2);

  otherwise
    error([ 'Partials for the game of two identical vehicles' ...
            ' only exist in dimensions 1-3' ]);
end
end
