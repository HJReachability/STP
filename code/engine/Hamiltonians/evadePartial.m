function alpha = evadePartial(t, data, derivMin, derivMax, schemeData, dim)

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

switch dim
  case 1
    alpha = abs(-v1 + vI*cos(g.xs{3})) + u1Max*abs(g.xs{2});
    alpha = alpha + ...
      dMax(1)*abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    
  case 2
    alpha = abs(vI*sin(g.xs{3})) + u1Max*abs(g.xs{1});
    alpha = alpha + ...
      dMax(1)*abs(derivMax{2}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    
  case 3
    alpha = u1Max + uIMax + dMax(2);
    
  otherwise
    error([ 'Partials for the game of two identical vehicles' ...
      ' only exist in dimensions 1-3' ]);
end
end