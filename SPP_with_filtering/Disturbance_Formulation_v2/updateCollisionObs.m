function vehicle = updateCollisionObs(g, tinit, tmax, tinit_index, vehicle)

if(tinit == tmax)
    return;
end

%---------------------------------------------------------------------------
% Update collision matrix based on the obstacle state measurement
tstep = vehicle.t_step;
mment = vehicle.mment(:,tinit_index);
% Measurement uncertainty model
% Circular model
if (strcmp(vehicle.state_uncertainty, 'circular'))
    mment_radius = vehicle.state_uncertainty_axis;
    collisionObs = sqrt((g.xs{1} - mment(1)).^2 + (g.xs{2} - mment(2)).^2 + + (g.xs{3} - mment(3)).^2) - mment_radius;
end
% Ellipsoidal model
if (strcmp(vehicle.state_uncertainty,'ellipsoid'))
    axis1_radius = vehicle.state_uncertainty_axis(1);
    axis2_radius = vehicle.state_uncertainty_axis(2);
    axis3_radius = vehicle.state_uncertainty_axis(3);
    if(tinit_index ==1) % Assuming very small uncertainty in the initial state; 2 is just chosen randomly
        f = 1;
        axis1_radius = axis1_radius/f;
        axis2_radius = axis2_radius/f;
        axis3_radius = axis3_radius/f;
    end
    collisionObs = sqrt((1/(axis1_radius)^2)*(g.xs{1} - mment(1)).^2 + (1/(axis2_radius)^2)*(g.xs{2} - mment(2)).^2 +...
        (1/(axis3_radius)^2)*(g.xs{3} - mment(3)).^2) - 1;
end

% Next we need to do filtering
vehicle.collisionmat(:,:,:,tinit_index) = shapeIntersection(collisionObs, vehicle.collisionmat(:,:,:,tinit_index));


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
% Project obstacle on 2D and then extend in 3D
[g2D, data2D] = proj2D(g, data, [0 0 1]);
vehicle.collisionmat(:,:,1:end,tinit_index) = repmat(data2D, [1,1,g.shape(3)]);

%---------------------------------------------------------------------------
% % What level set should we view?
% level = 0;
%
% % Visualize the 3D reachable set.
% displayType = 'surface';
%
% % Pause after each plot?
% pauseAfterPlot = 0;
%
% % Delete previous plot before showing next?
% deleteLastPlot = 1;
%
% % Visualize the angular dimension a little bigger.
% aspectRatio = [ 1 1 0.4 ];
%
% % Plot in separate subplots (set deleteLastPlot = 0 in this case)?
% useSubplots = 0;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @RASHamFunc;
schemeData.partialFunc = @RASPartialFunc;
schemeData.grid = g;
schemeData.target = vehicle.reach(:,:,:,end);

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
% % Initialize Display
% f = figure;
%
% % Set up subplot parameters if necessary.
% if(useSubplots)
%   rows = ceil(sqrt(plotSteps));
%   cols = ceil(plotSteps / rows);
%   plotNum = 1;
%   subplot(rows, cols, plotNum);
% end
%
% % h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);
% % camlight right;  camlight left;
% [g2D, data2D] = proj2D(g, data, [0 0 1]);
% h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');
%
% hold on;
% axis(g.axis);
% axis square
% drawnow;

%Initialize Display
figure(vehicle.mast_fig);
if(~isempty(vehicle.fig_hand))
    subplot(vehicle.fig_hand);
end
hold on;
[g2D, data2D] = proj2D(g, data, [0 0 1], vehicle.x(3,tinit_index));
[~, h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', vehicle.fig_color, 'Linestyle', '--');
drawnow;
hold off;

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
    
    % Input
    schemeData.U = vehicle.optU(:,:,:,tindex);
    
    % Take a timestep.
    [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
        integratorOptions, schemeData);
    tNow = t(end);
    
    % Get back the correctly shaped data array
    data = reshape(y, g.shape);
    
    % Project obstacle on 2D and then extend in 3D
    [g2D, data2D] = proj2D(g, data, [0 0 1]);
    tindex = tindex + 1;
    vehicle.collisionmat(:,:,1:end,tindex) = repmat(data2D, [1,1,g.shape(3)]);
    
    % Start plotting
    figure(vehicle.mast_fig);
    if(~isempty(vehicle.fig_hand))
        subplot(vehicle.fig_hand);
    end
    
    % Delete last visualization if necessary.
    delete(h2);
    
    %Create new visualization.
    hold on;
    [~,h2] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'color', vehicle.fig_color, 'Linestyle', '--');
    drawnow;
    %     pause;
    %
    %     % Keep the handle to the reachable set at the desired time
    %     if(abs((tmax + tinit - tNow)- vehicle.tplot) <= tstep/5 && (tinit == vehicle.tplot || tinit == 0))
    %        [~,vehicle.obs_hand] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', vehicle.fig_color);
    %     end
    %
    %     drawnow;
    %     hold off;
    
    %   if(pauseAfterPlot)
    %     % Wait for last plot to be digested.
    %     pause;
    %   end
    %
    %   % Get correct figure, and remember its current view.
    %   figure(f);
    %   [ view_az, view_el ] = view;
    %
    % %   % Delete last visualization if necessary.
    % %   if(deleteLastPlot)
    % %     delete(h);
    % %   end
    %
    %   % Move to next subplot if necessary.
    %   if(useSubplots)
    %     plotNum = plotNum + 1;
    %     subplot(rows, cols, plotNum);
    %   end
    %
    % %   %Create new visualization.
    % %   h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);
    %
    %   [g2D, data2D] = proj2D(g, data, [0 0 1]);
    %   h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');
    %   drawnow;
    %
    %   % Restore view.
    %   view(view_az, view_el);
end
% if(tinit > vehicle.tplot || tmax < vehicle.tplot)
%    vehicle.obs_hand = h2;
% end
delete(h2);

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = RASHamFunc(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.U;
d1 = schemeData.disturbance(1);
d2 = schemeData.disturbance(2);
d3 = schemeData.disturbance(3);
target = schemeData.target;

hamValue = v*deriv{1}.*cos(g.xs{3}) + d1*abs(deriv{1}) + v*deriv{2}.*sin(g.xs{3}) ...
    + d2*abs(deriv{2}) + deriv{3}.*w + d3*abs(deriv{3});

% Freeze dynamics inside target set
hamValue(target <= 0) = 0;

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
