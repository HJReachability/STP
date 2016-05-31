function hamValue = dubins3Dham_CCSObs(t, data, deriv, schemeData)
% hamValue = dubins3Dham_baseObs(t, data, deriv, schemeData)
%   Hamiltonian function for Dubins car used with the level set toolbox.
%   However the function is modified such that speed and turn rate range
%   can vary for every state. This function is specialized for computing
%   obsatcles using a centralized controller scheme (CCS).
%
% Inputs:
%   schemeData - problem parameters
%     .grid:   grid structure
%     .vrange: speed range of vehicle
%     .wMax:   Maximum turn rate
%     .uMode:  'min' or 'max' (defaults to 'min')
%     .dMax:   disturbance bounds (see below)
%     .dMode: 'min' or 'max' (defaults to 'max')
%     .tMode: 'backward' or 'forward'
%     .data_BRS: BRS data that can be used to compute the optimal control
%     for BRS
%     .reset_input: reset the input range 
%
% Dynamics:
%   \dot x      = v * cos(\theta) + d1
%   \dot y      = v * sin(\theta) + d2
%   \dot \theta = u + d3
% (d1, d2) \in disk of radius dMax(1)
% d3 \in [-dMax(2), dMax(2)]
%
% Somil Bansal, 2016-05-26

checkStructureFields(schemeData, 'grid', 'vrange', 'wrange', 'data_BRS', 'reset_input');

g = schemeData.grid;

%% Speed range
vrange = schemeData.vrange;
if isscalar(vrange)
  vrange{1} = vrange;
  vrange{2} = vrange;
end
vMin = vrange{1};
vMax = vrange{2};

%% Turnrate
wrange = schemeData.wrange;
if isscalar(wrange)
  wrange{1} = -wrange;
  wrange{2} = wrange;
end
wNom = 0.5*(wrange{1} + wrange{2});
wMax = wrange{2} - wNom;

%% Defaults: min over control, max over disturbance, backward reachable set
if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'max';
end

if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

%% Modify Hamiltonian control terms based on uMode
if strcmp(schemeData.uMode, 'min')
  % the speed when the determinant term (terms multiplying v) is positive
  % or negative
  v_when_det_pos = vMin;
  v_when_det_neg = vMax;
  
  % turn rate term
  wTerm = wNom.*deriv{3} - wMax.*abs(deriv{3});
elseif strcmp(schemeData.uMode, 'max')
  v_when_det_pos = vMax;
  v_when_det_neg = vMin;
  wTerm = wNom.*deriv{3} + wMax.*abs(deriv{3});
else
  error('Unknown uMode! Must be ''min'' or ''max''')
end

%% Hamiltonian control terms
% Speed control
hamValue = (deriv{1}.*cos(g.xs{3}) + deriv{2}.*sin(g.xs{3}) >= 0) .* ...
  (deriv{1}.*cos(g.xs{3}) + deriv{2}.*sin(g.xs{3})) .* v_when_det_pos + ...
  (deriv{1}.*cos(g.xs{3}) + deriv{2}.*sin(g.xs{3}) < 0) .* ...
  (deriv{1}.*cos(g.xs{3}) + deriv{2}.*sin(g.xs{3})) .* v_when_det_neg;

% turn rate control
hamValue = hamValue + wTerm;

%% Add disturbances if needed
if isfield(schemeData, 'dMax')
  dTerm = schemeData.dMax(1)*sqrt(deriv{1}.^2 + deriv{2}.^2) + ...
    schemeData.dMax(2)*abs(deriv{3});
  
  if strcmp(schemeData.dMode, 'min')
    hamValue = hamValue - dTerm;
  elseif strcmp(schemeData.dMode, 'max')
    hamValue = hamValue + dTerm;
  else
    error('Unknown dMode! Must be ''min'' or ''max''!')
  end
end

%% Backward or forward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
elseif ~strcmp(schemeData.tMode, 'forward')
  error('tMode must be ''forward'' or ''backward''!')
end
end
