function vehicle = updateReachSet(g, tinit, tmax, tinit_index, vehicle, obsVehicles)

%% To dos
% 1) Add fix_traj_Obs, fix_Obs to the Obstacle object. Don't forget to
% change the initial conditions appropriately.
% 2) Take care of fix obstacles in collisionObs and collisionmat

%% Start function

if(tinit == tmax)
    return;
end

%---------------------------------------------------------------------------
% Update obstacle shapes based on the new calculated obstacles
init_index = tinit_index;
tstep = vehicle.t_step;
steps = int64((tmax - tinit)/tstep);
end_index = int64(init_index + steps-1);

if(~isempty(obsVehicles)) % if there are any obstacles at all
    vnum = size(obsVehicles,2);
    obstacle = obsVehicles{1}.collisionmat(:,:,:,init_index:end_index+1);
    for i=2:vnum
        obstacle = shapeUnion(obstacle,obsVehicles{i}.collisionmat(:,:,:,init_index:end_index+1));
    end
else
    obstacle = 1e6*ones(g.shape);
    obstacle = repmat(obstacle, [ones(1,g.dim),steps+1]);
end

% Take the 2D projection of the above 3D obstacle, add the capture radius
% and then extend it in the third dimension
capture_radius = vehicle.capture_radius;
obs_len = end_index+2-init_index;
for i=1:obs_len
    [g2D, data2D] = proj2D(g, obstacle(:,:,:,i), [0 0 1]);
    dataout = addCRadius(g2D, data2D, capture_radius);
    obstacle(:,:,1:end,i) = repmat(dataout, [1,1,g.shape(3)]);
end

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
data = min(data, vehicle.reach(:,:,:,end));
data = max(data, -obstacle(:,:,:,mat_index-init_index+1));

P = extractCostates(g, data);
vehicle.optU(:,:,:,mat_index) = (P{3} >= 0) * (-vehicle.turnRate) + (P{3} < 0) * vehicle.turnRate;

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
schemeData.disturbance = vehicle.disturbance_mag;

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
if(~isempty(vehicle.fig_hand))
    subplot(vehicle.fig_hand);
end
hold on;
[g2D, data2D] = proj2D(g, data, [0 0 1], vehicle.x(3,tinit_index));
[~, h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'm', 'linestyle','-');
[g2D, data2D] = proj2D(g, obstacle(:,:,:,mat_index-init_index+1), [0 0 1], vehicle.x(3,tinit_index));
[~,h3] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'c', 'linestyle',':');
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
    
    % Update optimal control
    % Get gradients
    P = extractCostates(g, data);
    vehicle.optU(:,:,:,mat_index) = (P{3} >= 0) * (-vehicle.turnRate) + (P{3} < 0) * vehicle.turnRate;
    
    % Start plotting
    figure(vehicle.mast_fig);
    if(~isempty(vehicle.fig_hand))
        subplot(vehicle.fig_hand);
    end
    
    % Delete last visualization if necessary.
    delete(h2);
    delete(h3);
    
    %Create new visualization.
    hold on;
    [g2D, data2D] = proj2D(g, data, [0 0 1], vehicle.x(3,tinit_index));
    [~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'm', 'linestyle','-' );
    drawnow;
    [g2D, data2D] = proj2D(g, obstacle(:,:,:,mat_index-init_index+1), [0 0 1], vehicle.x(3,tinit_index));
    [~,h3] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'c', 'linestyle',':');
    drawnow;
    
    % Keep the handle to the reachable set at the desired time
    if(abs((tmax + tinit - tNow)- vehicle.tplot) <= tstep/5 && (abs(tinit - vehicle.tplot)<= tstep/5))
        plots_len = length(vehicle.plot_data);
        [g2D, data2D] = proj2D(g, data, [0 0 1], vehicle.x(3,tinit_index));
        [~,vehicle.reach_hand1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'm', 'linestyle', '--');
        drawnow;
        vehicle.plot_data{plots_len+1} = data2D; vehicle.plot_grid{plots_len+1} = g2D;
        [g2D, data2D] = proj2D(g, vehicle.cons_reach(:,:,:,tinit_index), [0 0 1], vehicle.x(3,tinit_index));
        [~,vehicle.reach_hand2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'm', 'linestyle', '--');
        drawnow;
        vehicle.plot_data{plots_len+2} = data2D; vehicle.plot_grid{plots_len+2} = g2D;
%         [g2D, data2D] = proj2D(g, obstacle(:,:,:,mat_index-init_index+1), [0 0 1], vehicle.x(3,tinit_index));
%         [~,vehicle.obs_hand1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'c', 'linestyle','-.');
%         drawnow;
    end
    
    % Taking advantage of the fact that obstacle is same for all angles (so
    % plotting at any angle should work)
    if(abs((tmax + tinit - tNow)- vehicle.tplot) <= tstep/5 && ((abs(tinit - vehicle.tplot)<= tstep/5) || tinit == 0))
        [g2D, data2D] = proj2D(g, obstacle(:,:,:,mat_index-init_index+1), [0 0 1]);
        [~,vehicle.obs_hand] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'c', 'linestyle','-.');
        drawnow;
        plots_len = length(vehicle.plot_data);
        vehicle.plot_data{plots_len+1} = data2D; vehicle.plot_grid{plots_len+1} = g2D;
    end
    
    hold off;
    
    % Record the initial TTR
    if (tinit_index == 1)
        location{1} = vehicle.x(1,1);
        location{2} = vehicle.x(2,1);
        location{3} = vehicle.x(3,1);
        [location_index, ~] = getCellIndexes(g, location);
        [index1, index2, index3] = ind2sub(g.shape, location_index);
        if (data(index1, index2, index3) <= 0)
            vehicle.initial_TTR = min(vehicle.initial_TTR, tNow);
        end
    end
    
    
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

% if(tinit > vehicle.tplot || tmax < vehicle.tplot)
%    vehicle.reach_hand = h2;
% end

if(tinit ~= 0)
    delete(h2);
end
delete(h3);

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
    + d2*abs(deriv{2}) - w*abs(deriv{3}) + d3*abs(deriv{3});
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
