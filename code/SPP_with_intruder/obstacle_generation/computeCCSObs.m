function [data, tau, extraOuts] = computeCCSObs( ...
  data0, tau, schemeData, minWith, extraArgs)
% This function uses the core structure of HJIPDE_solve with slight 
% modification so as to compute the obstacles generated under centralized  
% control scheme.

%% Default parameters
if numel(tau) < 2
  error('Time vector must have at least two elements!')
end

if nargin < 4
  minWith = 'zero';
end

if nargin < 5
  extraArgs = [];
end

extraOuts = [];
small = 1e-4;
colons = repmat({':'}, 1, schemeData.grid.dim);

%% Extract the information from extraargs
if isfield(extraArgs, 'visualize') && extraArgs.visualize
  % Extract the information about plotData
  if isfield(extraArgs, 'plotData')
    % Dimensions to visualize
    % It will be an array of 1s and 0s with 1s means that dimension should
    % be plotted.
    plotDims = extraArgs.plotData.plotDims;
    % Points to project other dimensions at. There should be an entry point
    % corresponding to each 0 in plotDims.
    projpt = extraArgs.plotData.projpt;
    % Initialize the figure for visualization
  else
    plotDims = ones(schemeData.grid.dim, 1);
    projpt = [];
  end
  
  f = figure;
  need_light = true;
end

% Extract the generation parameters
if isfield(extraArgs, 'genparams')
  gen_data = extraArgs.genparams.data;
  gen_resetR = extraArgs.genparams.reset_thresholds;
else
  error('Obstacle generation parameters needs to be mentioned!')
end

% Extract the information about targets
if isfield(extraArgs, 'targets')
  targets = extraArgs.targets;
end

%% SchemeFunc and SchemeData
schemeFunc = @termLaxFriedrichs;
% Extract accuracy parameter o/w set default accuracy
accuracy = 'veryHigh';
if isfield(schemeData, 'accuracy')
  accuracy = schemeData.accuracy;
end

%% Numerical approximation functions
dissType = 'global';
[schemeData.dissFunc, integratorFunc, schemeData.derivFunc] = ...
  getNumericalFuncs(dissType, accuracy);

%% Time integration
integratorOptions = odeCFLset('factorCFL', 0.8, 'stats', 'on', ...
  'singleStep', 'on');

startTime = cputime;

if schemeData.grid.dim == 1
  data = zeros(length(data0), length(tau));
else
  data = zeros([size(data0) length(tau)]);
end

data(colons{:}, 1) = data0;
schemeData.reset_input = false;
  
for i = 2:length(tau)
  y0 = data(colons{:}, i-1);
  y = y0(:);
  
  % Set the BRS for the optimal control
  data_BRS = gen_data(colons{:}, i-1);
  schemeData.data_BRS = data_BRS;

  tNow = tau(i-1);
  while tNow < tau(i) - small
    % Save previous data if needed
    if strcmp(minWith, 'zero')
      yLast = y;
    end
    
    [tNow, y] = feval(integratorFunc, schemeFunc, [tNow tau(i)], y, ...
      integratorOptions, schemeData);
    
    % Min with zero
    if strcmp(minWith, 'zero')
      y = min(y, yLast);
    end 
    
    % Min with targets
    if isfield(extraArgs, 'targets')
      if numDims(targets) == schemeData.grid.dim
        y = min(y, targets(:));
      else
        target_i = targets(colons{:}, i);
        y = min(y, target_i(:));
      end
    end
    
    % Reset the base obstacles if they go below the threshold
    schemeData.reset_input = false;
    [y0, update_flag] = check_obssize(schemeData.grid, y0, gen_resetR);
    if update_flag
      schemeData.reset_input = true;
    end
    
  end
  
  % Reshape value function
  data(colons{:}, i) = reshape(y, schemeData.grid.shape);
  
  %% If commanded, visualize the level set.
  if isfield(extraArgs, 'visualize') && extraArgs.visualize
    % Number of dimensions to be plotted and to be projected
    pDims = nnz(plotDims);
    projDims = length(projpt);
    
    % Basic Checks
    if(length(plotDims) ~= schemeData.grid.dim || ...
        projDims ~= (schemeData.grid.dim - pDims))
      error('Mismatch between plot and grid dimesnions!');
    end
    
    if (pDims >= 4 || schemeData.grid.dim > 4)
      error('Currently only 3D plotting upto 3D is supported!');
    end
    
    % Visualize the reachable set
    figure(f)
    
    if projDims == 0
      extraOuts.hT = visSetIm(schemeData.grid, data(colons{:}, i), ...
        'r', 0, [], false);
      
      if need_light && schemeData.grid.dim == 3
        camlight left
        camlight right
        need_light = false;
      end
    else
      [gProj, dataProj] = proj(schemeData.grid, data(colons{:}, i), ...
        1-plotDims, projpt);
      extraOuts.hT = visSetIm(gProj, dataProj, 'r', 0, [], false);
      
      if need_light && gProj.dim == 3
        camlight left
        camlight right
        need_light = false;
      end
      
    end
    
    drawnow;
  end
end

% Take intersection with the BRS. Intersection is outside of the for loop
% to make sure that the obstacle doesn't reduce significantly. This will
% give conservative obstacles though.
numObs = length(tau);
for i=1:numObs
data(colons{:}, i) = shapeIntersection(data(colons{:}, i), ...
  gen_data(colons{:}, i));
end

endTime = cputime;
fprintf('Total execution time %g seconds\n', endTime - startTime);
end

function [dissFunc, integratorFunc, derivFunc] = ...
  getNumericalFuncs(dissType, accuracy)
% Dissipation
switch(dissType)
  case 'global'
    dissFunc = @artificialDissipationGLF;
  case 'local'
    dissFunc = @artificialDissipationLLF;
  case 'locallocal'
    dissFunc = @artificialDissipationLLLF;
  otherwise
    error('Unknown dissipation function %s', dissFunc);
end

% Accuracy
switch(accuracy)
  case 'low'
    derivFunc = @upwindFirstFirst;
    integratorFunc = @odeCFL1;
  case 'medium'
    derivFunc = @upwindFirstENO2;
    integratorFunc = @odeCFL2;
  case 'high'
    derivFunc = @upwindFirstENO3;
    integratorFunc = @odeCFL3;
  case 'veryHigh'
    derivFunc = @upwindFirstWENO5;
    integratorFunc = @odeCFL3;
  otherwise
    error('Unknown accuracy level %s', accuracy);
end
end