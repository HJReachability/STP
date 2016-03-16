function FRS = computeFRS(FBC, IS, target, t0, visualize)
% RS = computeFRS(FBC, target, d, visualize)
% Computes the forward reachable set given a feedback control law that is 
% derived from a previously computed backwards reachable set
%
% Inputs: FBC       - feedback control law; a 4D matrix
%         IS        - initial state
%         visualize - set to true to visualize the reachable set computation
%
% Output: FRS       - forward reachable set
%                     .g    - grid structure
%                     .data - time-dependent reachable set
%                     .tau  - time stamps
%
% Mo Chen, 2016-02-02

if nargin < 5
  visualize = true;
end

%% Integration parameters.
tMax = FBC.tau(1);           % End time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

% Accuracy
accuracy = 'veryHigh';

%% Create initial conditions
% Variance of states
Xw = 1.5;
Yw = 1.5;
THETAw = 15*pi/180;

IC = sqrt((FBC.g.xs{1} - IS(1)).^2 / Xw^2 + ...
  (FBC.g.xs{2} - IS(2)).^2 / Yw^2 + ...
  (FBC.g.xs{3} - IS(3)).^2 / THETAw^2) - 1;

%% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;
schemeData.grid = FBC.g;

%% The Hamiltonian and partial functions need problem parameters.
schemeData.v = FBC.v;
schemeData.uMax = FBC.uMax;
schemeData.dMax = FBC.dMax;
schemeData.target = target;

%% Choose degree of dissipation.

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

%% Set up time approximation scheme.
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

%% Finalize initialization
tau = t0;
reach = IC;
data = IC;

if visualize
  figure;
  
  h = visualizeLevelSet(FBC.g, data, 'surface', 0);
  camlight right
  camlight left

  axis(FBC.g.axis);
  axis square
  drawnow;
end

%% Loop until tMax (subject to a little roundoff).
tNow = t0;
startTime = cputime;
while(tMax - tNow > small * tMax)
  %% Get feedback control
  ind = find(FBC.tau <= tNow, 1, 'first');

  % Feedback controller
  schemeData.Ut = FBC.U(:,:,:,ind);
% %   schemeData.It = FBC.I(:,:,:,ind);
%   for j = 1:g.dim
%     schemeData.Dt{j} = FBC.D{j}(:,:,:,ind);
%   end
%   
  % Reshape data array into column vector for ode solver call.
  y0 = data(:);
  
  % How far to step?
  tNext = tMax;
  tSpan = [tNow tNext];
  
  % Take a timestep.
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  % Get back the correctly shaped data array
  data = reshape(y, FBC.g.shape);
  tNow = t(end);
  
  reach = cat(4, reach, data);
  tau = cat(1, tau, tNow);
  
  if visualize
    delete(h);
    h = visualizeLevelSet(FBC.g, data, 'surface', 0);
    axis square
    drawnow;
  end
  
  if all(data(:) > 0)
    break
  end
end

%% Pack output
FRS.g = FBC.g;
FRS.data = reach;
FRS.tau = tau;

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
end

function hamValue = HamFunc(t, data, deriv, schemeData)
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% Hamiltonian for the forward reachable set

checkStructureFields(schemeData, 'grid', 'v', 'Ut', 'dMax', 'target');

g = schemeData.grid;
v = schemeData.v;
% Dt = schemeData.Dt;
Ut = schemeData.Ut;
% It = schemeData.It;
dMax = schemeData.dMax;
target = schemeData.target;

% Dynamics:
%  \dot{x}_1 = v * cos(x_3) + d_1
%  \dot{x}_2 = v * sin(x_3) + d_2
%  \dot{x}_3 = U(x_1, x_2, x_3) + d_3

% Dynamics without disturbances
hamValue = deriv{1}.*(v*cos(g.xs{3})) + deriv{2}.*(v*sin(g.xs{3})) + ...
  deriv{3}.*(Ut);

% Add disturbances
hamValue = hamValue + dMax(1)*abs(deriv{1}) + dMax(2)*abs(deriv{2}) + ...
  dMax(3)*abs(deriv{3});

% Freeze dynamics inside target set
hamValue(target <= 0) = 0;

end

function alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% air3DPartialFunc: Hamiltonian partial fcn for 3D collision avoidance example.
%
% alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
%
% This function implements the partialFunc prototype
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
% Ian Mitchell 3/26/04

checkStructureFields(schemeData, 'grid', 'v', 'uMax', 'dMax');

g = schemeData.grid;
v = schemeData.v;
uMax = schemeData.uMax;
dMax = schemeData.dMax;

switch dim
  case 1
    alpha = abs(v*cos(g.xs{3})) + dMax(1);
    
  case 2
    alpha = abs(v*sin(g.xs{3})) + dMax(2);
    
  case 3
    alpha = uMax  + dMax(3);

end
end
