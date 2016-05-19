function alpha = dubins3Dpartial(t, data, derivMin, derivMax, schemeData, dim)

checkStructureFields(schemeData, 'grid', 'speed');

g = schemeData.grid;
v = schemeData.speed;

switch dim
  case 1
    alpha = v*abs(cos(g.xs{3}));
    
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + ...
        dMax(1) * abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    end
    
  case 2
    alpha = v*abs(sin(g.xs{3})); 
    
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + ...
        dMax(1) * abs(derivMax{2}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    end
    
  case 3
    alpha = 0;
    if isfield(schemeData, 'uMax')
      uMax = schemeData.uMax;
      alpha = alpha + uMax;
    end
    
    if isfield(schemeData, 'dMax')
      dMax = schemeData.dMax;
      alpha = alpha + dMax(2);
    end
end
end