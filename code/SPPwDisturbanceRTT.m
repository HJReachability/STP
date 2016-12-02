function SPPwDisturbanceRTT(problem_name, extraArgs)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 2
  extraArgs = [];
end

if isfield(extraArgs, 'SPPP')
  SPPP = extraArgs.SPPP;
else
  SPPP = SPPProblem(problem_name, extraArgs);
end

% % RTT parameters for ACC and TCST papers
% vReserved = [0.25 -0.25];
% wReserved = -0.4;
% trackingRadius = 0.075;

SPPP.computeRTTRS();
SPPP.computeNIRS();
SPPP.simulateNI();
end