function hamValue = dubins3Dham(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'speed');

g = schemeData.grid;
v = schemeData.speed;

%% Dynamics:
% \dot x = v * cos(\theta) + d1
% \dot y = v * sin(\theta) + d2
% \dot \theta = u + d3

% (d1, d2) \in disk of radius dMax(1)
% d3 \in [-dMax(2), dMax(2)]

% Dynamics without control and disturbances
hamValue = v*deriv{1}.*cos(g.xs{3}) + v*deriv{2}.*sin(g.xs{3});

%% Add control if needed
if isfield(schemeData, 'uMax')
  uMax = schemeData.uMax;
  
  % Default min over u
  if ~isfield(schemeData, 'uMode')
    schemeData.uMode = 'min';
  end
  uMode = schemeData.uMode;
  
  uTerm = uMax*abs(deriv{3});
  if strcmp(uMode, 'min') 
    hamValue = hamValue - uTerm;
  elseif strcmp(uMode, 'max')
    hamValue = hamValue + uTerm;
  else
    error('Unknown uMode! Must be ''min'' or ''max''')
  end
end

%% Add disturbances if needed
if isfield(schemeData, 'dMax')
  dMax = schemeData.dMax;
  
  % Default max over d
  if ~isfield(schemeData, 'dMode')
    schemeData.dMode = 'max';
  end
  dMode = schemeData.dMode;
  
  dTerm = dMax(1) * sqrt(deriv{1}.^2 + deriv{2}.^2) + abs(deriv{3}) * dMax(2);
  if strcmp(dMode, 'min') 
    hamValue = hamValue - dTerm;
  elseif strcmp(dMode, 'max')
    hamValue = hamValue + dTerm;
  else
    error('Unknown dMode! Must be ''min'' or ''max''!')
  end
end

%% Backward or forward reachable set
if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
elseif strcmp(schemeData.tMode, 'forward')
  % Nothing to do here
else
  error('tMode must be ''forward''!')
end
end
