function vehicle = updateReachSet(g, tinit, tmax, tinit_index, vehicle, obsVehicles)

%% To dos
% 1) Add fix_traj_Obs, fix_Obs to the Obstacle object. Don't forget to
% change the initial conditions appropriately.
% 2) Take care of fix obstacles in collisionObs and collisionmat

%% Start function

%---------------------------------------------------------------------------
% Update obstacle shapes based on the new calculated obstacles
init_index = tinit_index;
tstep = vehicle.t_step;
steps = (tmax - tinit)/tstep;
end_index = int64(init_index + steps-1);

if(nargin==6) % if there are any obstacles at all
    vnum = size(obsVehicles,2);
    obstacle = obsVehicles{1}.collisionmat(:,:,:,init_index:end_index);
    for i=2:vnum
        obstacle = shapeUnion(obstacle,obsVehicles{i}.collisionmat(:,:,:,init_index:end_index));
    end
else
    obstacle = 1e6*ones(g.shape);
    obstacle = repmat(obstacle, [ones(1,g.dim),steps]);
end

% Take the 2D projection of the above 3D obstacle and then extend it in the
% third dimension
% To be implemented

%---------------------------------------------------------------------------
% Integration parameters.

t0 = tinit;                  % Start time.
tMax = tmax;                 % End time.

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% How much accuracy?
accuracy = vehicle.accuracy;

% % Plotting parameters
% plotSteps = 9;               % How many intermediate plots to produce?
% singleStep = 0;              % Plot at each timestep (overrides tPlot).
%
% % Period at which intermediate plots should be produced.
% tPlot = (tMax - t0) / (plotSteps - 1);

%---------------------------------------------------------------------------
% create initial conditions
% Creating an index that will take care of time reversal
mat_index = end_index+1;
data = vehicle.reach(:,:,:,mat_index);

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

% The Hamiltonian and partial functions need problem parameters.
schemeData.velocity = vehicle.velocity;
schemeData.turnRate = vehicle.turnRate;
schemeData.input_uncertainty = vehicle.input_uncertainty_axis;

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

% if(singleStep)
%     integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
% end

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
% h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(t0) ]);
% camlight right;  camlight left;
% % [g2D, data2D] = proj2D(g, data, [0 0 1], pi);
% % h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');
%
% hold on;
% axis(g.axis);
% axis square
% drawnow;

% Initialize Display
figure(vehicle.mast_fig);
subplot(vehicle.fig_hand);
hold on;
[g2D, data2D] = proj2D(g, data, [0 0 1], pi);
[~, h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', vehicle.fig_color);
drawnow;
hold off;

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;

while(tMax - tNow > small * tMax)
    
    % Reshape data array into column vector for ode solver call.
    y0 = data(:);
    
    % How far to step?
    tSpan = [ tNow, min(tMax, tNow + tstep) ];
    
    % nominal input
    schemeData.u_nom = vehicle.u_nom(mat_index-1);
    
    % Take a timestep.
    [ t y ] = feval(integratorFunc, schemeFunc, tSpan, y0,...
        integratorOptions, schemeData);
    tNow = t(end);
    
    % Get back the correctly shaped data array
    data = reshape(y, g.shape);
    
    % Remove obstacle from the reachable set and store it
    mat_index = mat_index - 1;
    data = min(data, vehicle.reach(:,:,:,end));
    data = max(data, -obstacle(:,:,:,mat_index-init_index+1));
    vehicle.reach(:,:,:,mat_index) = data;
    
    % Start plotting
    figure(vehicle.mast_fig);
    subplot(vehicle.fig_hand);
    
    % Delete last visualization if necessary.
    delete(h2);
    
    %Create new visualization.
    hold on;
    [g2D, data2D] = proj2D(g, data, [0 0 1], 0);
    [~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', vehicle.fig_color);
    drawnow;
    hold off;
    
    %   if(pauseAfterPlot)
    %     % Wait for last plot to be digested.
    %     pause;
    %   end
    %
    %   % Get correct figure, and remember its current view.
    %   figure(f);
    %   [ view_az, view_el ] = view;
    %
    %   % Delete last visualization if necessary.
    %   if(deleteLastPlot)
    %     delete(h);
    %   end
    %
    %   % Move to next subplot if necessary.
    %   if(useSubplots)
    %     plotNum = plotNum + 1;
    %     subplot(rows, cols, plotNum);
    %   end
    %
    %   %Create new visualization.
    %   h = visualizeLevelSet(g, data, displayType, level, [ 't = ' num2str(tNow) ]);
    %
    % %   [g2D, data2D] = proj2D(g, data, [0 0 1], pi);
    % %   h = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'b');
    %   drawnow;
    %
    %   % Restore view.
    %   view(view_az, view_el);
end

vehicle.last_hand = h2;
endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = RASHamFunc(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
v = schemeData.velocity;
w = schemeData.input_uncertainty;
u_nom = schemeData.u_nom;

hamValue = v*deriv{1}.*cos(g.xs{3}) + v*deriv{2}.*sin(g.xs{3}) ...
    - w*abs(deriv{3}) + u_nom*deriv{3};
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
w = schemeData.input_uncertainty;
u_nom = schemeData.u_nom;

switch dim
    case 1
        alpha = v*abs(cos(g.xs{3}));
        
    case 2
        alpha = v*abs(sin(g.xs{3}));
        
    case 3
        %        alpha = abs(u_nom) + w*abs(derivMax{3});
        alpha = abs(u_nom) + w;
end
