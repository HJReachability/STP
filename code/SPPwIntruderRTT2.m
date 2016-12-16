function SPPwIntruderRTT2(problem_name, extraArgs)
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

% % RTT parameters
% vReserved = [1.5 -0.5];
% wReserved = -0.8;
% trackingRadius = 0.4;
% 
% SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);

Qintr = Plane([0; 0; 0], SPPP.wMaxA, SPPP.vRangeA, SPPP.dMaxA);
tIAT = 10;
SPPP.computeCARS(Qintr, tIAT);

SPPP.computeMinMinBRS;

keyboard
SPPP.computeRawAugObs2;


SPPP.computeBRRS2;
SPPP.simulateBR2;
SPPP.computeARRS2;
SPPP.simulateAR;
SPPP.simulateFull;
end