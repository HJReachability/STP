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

targetR = 0.1;
SPPP = SPPProblem(initStates, targetCenters, targetR);

SPPP.computeRTTRS;
SPPP.computeCARS;
SPPP.computeRawAugObs;
SPPP.computeBRRS;
SPPP.simulateBR;
SPPP.computeARRS;
SPPP.simulateAR;
SPPP.simulateFull;
end