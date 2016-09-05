function SPPwIntruderRTT()
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

initStates = { ...
  [-0.5; 0;  0]; ...
  [ 0.5; 0; -pi]; ...
  [-0.6; 0.6; -pi/4]; ...
  [ 0.6; 0.6; -3*pi/4]};

targetCenters = { ...
  [ 0.7;  0.2; 0]; ...
  [-0.7;  0.2; 0]; ...
  [ 0.7; -0.7; 0]; ...
  [-0.7; -0.7; 0]};

targetR = 0.15;

% Vehicle parameters
vehParams.vRangeA = [0.1 1];
vehParams.wMaxA = 1;
vehParams.dMaxA = [0.1 0.2];

% RTT parameters
vReserved = [0.3 -0.3];
wReserved = -0.4;
trackingRadius = 0.075;

SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams);

fprintf('Enter any modifications to the SPPProblem...\n')
keyboard

SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);
SPPP.computeCARS;
SPPP.computeRawAugObs;
SPPP.computeBRRS;
SPPP.simulateBR;
SPPP.computeARRS;
SPPP.simulateAR;
SPPP.simulateFull;
end