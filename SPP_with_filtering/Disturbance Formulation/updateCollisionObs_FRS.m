function vehicle = updateCollisionObs(g, tinit, tmax, tinit_index, vehicle)

%% Start function

%---------------------------------------------------------------------------
% Update collision matrix based on the obstacle state measurement
capture_radius = vehicle.capture_radius;
tstep = vehicle.t_step;
mment = vehicle.mment;
% Measurement uncertainty model
% Circular model
if (strcmp(vehicle.state_uncertainty, 'circular'))
    mment_radius = vehicle.state_uncertainty_axis;
    collisionObs = sqrt((g.xs{1} - mment(1,1)).^2 + (g.xs{2} - mment(1,2)).^2 + + (g.xs{3} - mment(1,3)).^2) - (mment_radius+capture_radius);
end
% Ellipsoidal model
if (strcmp(vehicle.state_uncertainty,'ellipsoid'))
    axis1_radius = vehicle.state_uncertainty_axis(1);
    axis2_radius = vehicle.state_uncertainty_axis(2);
    axis3_radius = vehicle.state_uncertainty_axis(3);
    collisionObs = sqrt((1/(axis1_radius+capture_radius)^2)*(g.xs{1} - mment(1,1)).^2 + (1/(axis2_radius+capture_radius)^2)*(g.xs{2} - mment(1,2)).^2 +...
        (1/(axis3_radius+capture_radius)^2)*(g.xs{3} - mment(1,3)).^2) - 1;
end

% Next we need to do filtering
vehicle.collisionmat(:,:,:,tinit_index) = shapeIntersection(collisionObs, vehicle.collisionmat(:,:,:,tinit_index));
%---------------------------------------------------------------------------

%---------------------------------------------------------------------------
% Integration parameters.

t0 = tinit;                   % Start time.
plotSteps = 9;               % How many intermediate plots to produce?
tMax = tmax;                  % End time.
singleStep = 0;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% How much accuracy?
accuracy = vehicle.accuracy;

%---------------------------------------------------------------------------
% create initial conditions
data = vehicle.collisionmat(:,:,:,tinit_index);

%---------------------------------------------------------------------------
% What level set should we view?
level = 0;

% Visualize the 3D reachable set.
displayType = 'surface';

% Pause after each plot?
pauseAfterPlot = 0;

% Delete previous plot before showing next?
deleteLastPlot = 1;

% Visualize the angular dimension a little bigger.
aspectRatio = [ 1 1 0.4 ];

% Plot in separate subplots (set deleteLastPlot = 0 in this case)?
useSubplots = 0;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @RASHamFunc;
schemeData.partialFunc = @RASPartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters. 
schemeData.velocity = vehicle.velocity;
schemeData.turnRate = vehicle.turnRate;
schemeData.disturbance = vehicle.disturbance_mag;
%---------------------------------------------------------------------------

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

%---------------------------------------------------------------------------

%---------------------------------------------------------------------------
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

%---------------------------------------------------------------------------
% Initialize Display
f = figure;

% Set up subplot parameters if necessary.
if(useSubplots)
  rows = ceil(sqrt(plotSteps));
  cols = ceil(plotSteps / rows);
  plotNum = 1;
  subplot(rows, cols, plotNum);
end

% h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);
% camlight right;  camlight left;
[g2D, data2D] = proj2D(g, data, [0 0 1]);
h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');

hold on;
axis(g.axis);
axis square
drawnow;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
tindex = tinit_index;
startTime = cputime;

while(tMax - tNow > small * tMax)

  % Reshape data array into column vector for ode solver call.
  y0 = data(:);

  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tstep) ];
  
  % Take a timestep.
  [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
                  integratorOptions, schemeData);
  tNow = t(end);

  % Get back the correctly shaped data array
  data = reshape(y, g.shape);
  tindex = tindex + 1;
  vehicle.collisionmat(:,:,:,tindex) = data;
  
  if(pauseAfterPlot)
    % Wait for last plot to be digested.
    pause;
  end

  % Get correct figure, and remember its current view.
  figure(f);
  [ view_az, view_el ] = view;

%   % Delete last visualization if necessary.
%   if(deleteLastPlot)
%     delete(h);
%   end

  % Move to next subplot if necessary.
  if(useSubplots)
    plotNum = plotNum + 1;
    subplot(rows, cols, plotNum);
  end

%   %Create new visualization.
%   h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);

  [g2D, data2D] = proj2D(g, data, [0 0 1]);
  h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');
  drawnow;
  
  % Restore view.
  view(view_az, view_el);
end

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
d1 = schemeData.disturbance(1);
d2 = schemeData.disturbance(2);
d3 = schemeData.disturbance(3);

hamValue = v*deriv{1}.*cos(g.xs{3}) + d1*abs(deriv{1}) + v*deriv{2}.*sin(g.xs{3}) ...
           + d2*abs(deriv{2}) + w*abs(deriv{3}) + d3*abs(deriv{3});

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
d1 = schemeData.disturbance(1);
d2 = schemeData.disturbance(2);
d3 = schemeData.disturbance(3);

switch dim
    case 1
        alpha = v*abs(cos(g.xs{3})) + d1;
        
    case 2
        alpha = v*abs(sin(g.xs{3})) + d2;
        
    case 3
        alpha = d3 + w;
end
