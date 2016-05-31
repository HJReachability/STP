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

checkStructureFields(schemeData, 'grid', 'vrange', 'wrange');

g = schemeData.grid;
vrange = schemeData.vrange;
if isscalar(vrange)
  vrange{1} = vrange;
  vrange{2} = vrange;
end
vMax = vrange{2};

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
    wrange = schemeData.wrange;
    if isscalar(wrange)
      wrange{1} = -wrange;
      wrange{2} = wrange;
    end
    alpha = abs(wrange{2});
    
    % Disturbance if needed
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + dMax(2);
    end
end
end