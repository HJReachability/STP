function SPPwDisturbanceRTT(problem_name, RTTRS_filename)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

extraArgs = [];
if nargin > 1
  extraArgs.RTTRS_filename = RTTRS_filename;
end

SPPP = SPPProblem(problem_name, extraArgs);

% % RTT parameters for ACC and TCST papers
% vReserved = [0.25 -0.25];
% wReserved = -0.4;
% trackingRadius = 0.075;

SPPP.computeRTTRS();
SPPP.computeNIRS();
SPPP.simulateNI();
end