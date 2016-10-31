function SPPwIntruderRTT2(SPPP)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 1
  initStates = { ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4] ...
    };
  
  targetCenters = { ...
    [100; 100; 0]; ...
    [300; 100; 0]; ...
    [100; 100; 0]; ...
    [100; 300; 0] ...
    };
  
  targetR = 50;
  
  % Vehicle parameters
  vehParams.vRangeA = [0.25 2.5];
  vehParams.wMaxA = 2;
  vehParams.dMaxA = 0.2*[max(vehParams.vRangeA) vehParams.wMaxA];
  
  % Grid parameters
  gridParams.min = [0; 0; 0];
  gridParams.max = [500; 500; 2*pi];
  gridParams.N = [95; 95; 95];
  
  SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, gridParams);
  fprintf('Enter any modifications to the SPPProblem...\n')
  keyboard
end

% RTT parameters
vReserved = [1.5 -0.5];
wReserved = -0.8;
trackingRadius = 1;

SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);
SPPP.computeCARS;
SPPP.computeRawAugObs2;
SPPP.computeBRRS2;
SPPP.simulateBR2;
SPPP.computeARRS2;
SPPP.simulateAR;
SPPP.simulateFull;
end