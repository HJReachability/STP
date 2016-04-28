function dataOut = addCRadius(gIn, dataIn, radius)
% dataOut = addCRadius(gIn, dataIn, radius)
%
% Expands the 2D shape given by gIn and dataIn by radius units all around
%
% Mo Chen, 2016-02-09

%% Numerical scheme parameters
schemeData.dissFunc = @artificialDissipationGLF;
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;

schemeData.grid = gIn;
schemeData.velocity = radius;

integratorOptions = odeCFLset('factorCFL', 0.5, 'stats', 'on');

%% Solve!
schemeData.derivFunc = @upwindFirstWENO5;
integratorFunc = @odeCFL3;

% Reshape data array into column vector for ode solver call.
y0 = dataIn(:);

tSpan = [0 1];

% Take a timestep.
[~, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
  integratorOptions, schemeData);

% Get back the correctly shaped data array
dataOut = reshape(y, gIn.shape);

end

function hamValue = HamFunc(t, data, deriv, schemeData)
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% Hamiltonian function

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
R = schemeData.velocity;

% Dynamics without disturbances
hamValue = R*sqrt(deriv{1}.^2 + deriv{2}.^2);

end

function alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% 
% Partial function

checkStructureFields(schemeData, 'grid', 'velocity');

g = schemeData.grid;
R = schemeData.velocity;


switch dim
  case 1
    alpha = R*abs(derivMax{1}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);
    
  case 2
    alpha = R*abs(derivMax{2}) / sqrt(derivMax{1}.^2 + derivMax{2}.^2);

end
end