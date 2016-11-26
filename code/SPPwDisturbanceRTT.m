function SPPwDisturbanceRTT(SPPP)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 1
%   extraArgs.RTTRS_filename = 'RTTRS11.mat';
%   SPPP = SPPProblem('SF_dstb_11', extraArgs);
  
  extraArgs.RTTRS_filename = 'RTTRS6.mat';
  SPPP = SPPProblem('SF_dstb_6', extraArgs);  
  
end

% % RTT parameters for ACC and TCST papers
% vReserved = [0.25 -0.25];
% wReserved = -0.4;
% trackingRadius = 0.075;

SPPP.computeRTTRS();
SPPP.computeNIRS();
SPPP.simulateNI();
end