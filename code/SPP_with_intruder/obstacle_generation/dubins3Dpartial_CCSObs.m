function alpha = dubins3Dpartial_CCSObs( ...
  t, data, derivMin, derivMax, schemeData, dim)
% Inputs:
%   schemeData - problem parameters
%     .grid: grid structure
%     .vrange: speed range of vehicle
%     .wrange:  turn rate range
%     .dMax:  disturbance bounds (see below)
%
% Dynamics:
%   \dot x      = v * cos(\theta) + d1
%   \dot y      = v * sin(\theta) + d2
%   \dot \theta = u + d3
% (d1, d2) \in disk of radius dMax(1)
% d3 \in [-dMax(2), dMax(2)]
%
% Somil Bansal, 2016-05-26

checkStructureFields(schemeData, 'grid', 'vrange', 'wMax', 'data_BRS', 'reset_input');

g = schemeData.grid;

%% Compute the input range
% Speed range
vrange = schemeData.vrange;
if isscalar(vrange)
  vrange = [vrange vrange];
end

% TurnRate
wMax = schemeData.wMax;

if(schemeData.reset_input)
  % Use the full range
  velocity{1} = vrange(1);
  velocity{2} = vrange(2);
  turnR{1} = -wMax;
  turnR{2} = wMax;
else
  % Extract the optimal input
  P = extractCostates(g, schemeData.data_BRS);
  optW = (P{3} >= 0) * -wMax + (P{3} < 0) * wMax;
  optV = ((P{1}.*cos(g.xs{3}) + P{2}.*sin(g.xs{3})) >= 0) * vrange(1)...
    + ((P{1}.*cos(g.xs{3}) + P{2}.*sin(g.xs{3})) < 0) * vrange(2);
  velocity{1} = optV;
  velocity{2} = optV;
  turnR{1} = optW;
  turnR{2} = optW;
end

% Extract the inputs
vMax = abs(velocity{2});
wMax = abs(turnR{2});

switch dim
  case 1
    % Control
    alpha = vMax.*abs(cos(g.xs{3}));
    
    % Disturbance if needed
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + ...
        dMax(1) * abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    end
    
  case 2
    % Control
    alpha = vMax.*abs(sin(g.xs{3})); 
    
    % Disturbance if needed
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + ...
        dMax(1) * abs(derivMax{2}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    end
    
  case 3
    % Control
    alpha = wMax;
    
    % Disturbance if needed
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + dMax(2);
    end
end
end