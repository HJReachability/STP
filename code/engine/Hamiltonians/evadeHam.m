function hamValue = evadeHam(t, data, deriv, schemeData)

checkStructureFields(schemeData, 'grid', 'v1', 'vI', ...
  'u1Max', 'uIMax', 'd1Max', 'dIMax');

g = schemeData.grid;
v1 = schemeData.v1;
vI = schemeData.vI;
u1Max = schemeData.u1Max;
uIMax = schemeData.uIMax;
d1Max = schemeData.d1Max;
dIMax = schemeData.dIMax;
dMax = d1Max + dIMax;

% implements equation (3.3) from my thesis term by term
%   with allowances for \script A and \script B \neq [ -1, +1 ]
%   where deriv{i} is p_i
%         x_r is grid.xs{1}, y_r is grid.xs{2}, \psi_r is grid.xs{3}
%         v_a is velocityA, v_b is velocityB,
%         \script A is inputA and \script B is inputB

% Controls
hamValue = deriv{1}.*(-v1 + vI*cos(g.xs{3})) + deriv{2}.*(vI*sin(g.xs{3})) + ...
  u1Max * abs(g.xs{2}.*deriv{1} - g.xs{1}.*deriv{2} - deriv{3}) + ...
  -uIMax * abs(deriv{3});

% Disturbances
hamValue = hamValue + ...
  -dMax(1) * sqrt(deriv{1}.^2 + deriv{2}.^2) -dMax(2)*abs(deriv{3});

hamValue = -hamValue;
end


