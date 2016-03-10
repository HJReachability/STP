function [ data, g, data0, tau ] = coop4D(accuracy)
% reachAvoid4D: solves the 1 vs. 1 game between an attacker and a defender
%
%   [ data, g, data0 ] = reachAvoid4D(accuracy)
%
% Solves the 2-player reachavoid game for all possible joint initial
% configurations. For more details, see ACC 2014 paper.
%
% Code is adapted from Ian Mitchell's level set toolbox
%
% Input Parameters:
%
%   accuracy: Controls the order of approximations.
%     'low': Use odeCFL1 and upwindFirstFirst.
%     'medium': Use odeCFL2 and upwindFirstENO2 (default).
%     'high': Use odeCFL3 and upwindFirstENO3.
%     'veryHigh': Use odeCFL3 and upwindFirstWENO5.
%
%   extraAvoid: additional avoid set for the attacker
%       default avoid sets are specified in the game setup .m files
%
%   extraAvoid_g: grid structure associated with extraAvoid
%
% Output Parameters:
%
%   data: Implicit surface function at t_max.
%
%   g: Grid structure on which data was computed.
%
%   data0: Implicit surface function at t_0.

% Copyright 2004 Ian M. Mitchell (mitchell@cs.ubc.ca).
% This software is used, copied and distributed under the licensing
%   agreement contained in the file LICENSE in the top directory of
%   the distribution.
%
% Ian Mitchell, 3/26/04
% Mo Chen, Sept. 25, 2013
%
%---------------------------------------------------------------------------
% You will see many executable lines that are commented out.
%   These are included to show some of the options available; modify
%   the commenting to modify the behavior.

%---------------------------------------------------------------------------
% Default parameters
% if(nargin < 1), accuracy = 'low'; end
% if(nargin < 1), accuracy = 'medium'; end
if(nargin < 1), accuracy = 'veryHigh'; end

%---------------------------------------------------------------------------
% Integration parameters.
tMax = 4;                   % End time.
plotSteps = 10;              % How many intermediate plots to produce?
t0 = 0;                      % Start time.

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% Delete previous plot before showing next?
deleteLastPlot = 1;


%---------------------------------------------------------------------------
% How many grid cells?
Nx = 45;
% Nx = 31;

% Create the computation grid.
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

%---------------------------------------------------------------------------
% Load game domain information and player positions
% run midTarget_LObs
% run nonconvexExample
run('Game Data\OLGameCoop2D')

% Initial conditions
data = targetLoc;
avoid = shapeUnion(collision,obs4D);

data0 = data;

%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @coop4DHamFunc;
schemeData.partialFunc = @coop4DPartialFunc;
schemeData.grid = g;

% The Hamiltonian and partial functions need problem parameters.
schemeData.velocity1 = velocity1;
schemeData.velocity2 = velocity2;

%---------------------------------------------------------------------------
% Choose degree of dissipation.

switch(dissType)
    case 'global',      schemeData.dissFunc = @artificialDissipationGLF;
    case 'local',       schemeData.dissFunc = @artificialDissipationLLF;
    case 'locallocal',  schemeData.dissFunc = @artificialDissipationLLLF;
    otherwise,          error('Unknown dissipation function %s', dissFunc);
end
%---------------------------------------------------------------------------

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.9, 'stats', 'on');

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

% Set up masking so that the reachable set does not propagate through the
% avoid set
schemeData.maskData = -avoid(:);
schemeData.maskFunc = @max;

% Let the integrator know what function to call.
integratorOptions = odeCFLset(integratorOptions, 'postTimestep', @postTimestepMask);

% schemeData.grid = g;
% integratorOptions = odeCFLset(integratorOptions, 'postTimestep', @postTimestepReinit);
integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');

%---------------------------------------------------------------------------
% Initialize Display
%keyboard
f1 = figure;
f2 = figure;

hs = plotReachSets(g,g2D,target2D,obs2D,data,...
    x1_init,x2_init,captureRadius,f1,f2, dims_1, dims_2, dom_map);

%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;

tic;
tau = tNow;
P = extractCostates(g,data);
save(['coop4D_' num2str(length(tau))],'data','P','g','tau')
toc;

return
while(tMax - tNow > small * tMax)
%     keyboard
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
    
    tic;
    tau = cat(1,-tNow,tau);
    P = extractCostates(g,data);
    save(['coop4D_' num2str(length(tau))],'data','P','g','tau')
    toc;

    % Get correct figure, and remember its current view.
    figure(f1);
    [ view_az, view_el ] = view;
    
    % Delete last visualization if necessary.
    if(deleteLastPlot)
        for i = 1:length(x2_init), delete(hs.hd{i}); end
        for i = 1:length(x1_init), delete(hs.ha{i}); end        
    end
    
    % Create new visualization.
    hs = plotReachSets(g,g2D,target2D,obs2D,data,...
        x1_init,x2_init,captureRadius,f1,f2, dims_1, dims_2, dom_map);
    
    % Restore view.
    view(view_az, view_el);
    
    x1 = x1_init{1};
    x2 = x2_init{1};
    if eval_u(g,data,[x1 x2])<0
        break;
    end
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
return
% Figure legends
figure(f1)
subplot(spRd, spCd, length(x2_init))
legend([hsd.ht hsd.ho hsd.hxds{1} hsd.hxas{1} hd{end}], ...
    {'Target','Obstacle','Defender','Attacker','Reachable Set'})
set(legend,'units','pixels','position',[560 100 200 125])

figure(f2)
subplot(spRa, spCa, length(x1_init))
legend([hsa.ht hsa.ho hsa.hxds{1} hsa.hxas{1} ha{end}], ...
    {'Target','Obstacle','Defender','Attacker','Reachable Set'})
set(legend,'units','pixels','position',[560 100 200 125])


% --------------------------------------------------------------------------
% Summary figure
fprintf('Plotting summary\n');
f3 = figure;
figure(f3)
title('Game Summary')

% Show target
tara = visualizeLevelSet(g2D, target2D, displayType, level);
set(tara,'color',[0 0.5 0],'linewidth',2)
hold on

% Show obstacles
oa = visualizeLevelSet(g2D, obs, displayType, level);
set(oa,'color','k','linewidth',2)

end


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = coop4DHamFunc(t, data, deriv, schemeData)
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

checkStructureFields(schemeData, 'velocity1', 'velocity2');

v1 = schemeData.velocity1; % Defender velocity
v2 = schemeData.velocity2; % Attacker velocity

hamValue = -( - v1*sqrt(deriv{1}.^2 + deriv{2}.^2) ...
    - v2*sqrt(deriv{3}.^2 + deriv{4}.^2)   );
end


%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = coop4DPartialFunc(t, data, derivMin, derivMax, schemeData, dim)
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

checkStructureFields(schemeData, 'velocity1', 'velocity2');

v1 = schemeData.velocity1;
v2 = schemeData.velocity2;

% Speeds of players
norm_v1 = sqrt(derivMax{1}.^2 + derivMax{2}.^2); % attacker
norm_v2 = sqrt(derivMax{3}.^2 + derivMax{4}.^2); % defender

switch dim
    case 1
        alpha = v1*abs(derivMax{1})./norm_v1;
        
    case 2
        alpha = v1*abs(derivMax{2})./norm_v1;
        
    case 3
        alpha = v2*abs(derivMax{3})./norm_v2;
        
    case 4
        alpha = v2*abs(derivMax{4})./norm_v2;
        
    otherwise
        error([ 'Partials for the two player reach-avoid game' ...
            ' only exist in dimensions 1-4' ]);
end
end
