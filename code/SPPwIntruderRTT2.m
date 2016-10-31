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
  
  SPPP.tMin = -100;
  SPPP.dt = 1;
  SPPP.Rc = 1;
  
  fprintf('Enter any modifications to the SPPProblem...\n')
  keyboard
end

% RTT parameters
vReserved = [1.5 -0.5];
wReserved = -0.8;
trackingRadius = 0.25;

SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);

Qintr = Plane([0; 0; 0], vehParams.wMaxA, vehParams.vRangeA, vehParams.dMaxA);
tIAT = 10;
SPPP.computeCARS(Qintr, tIAT);

SPPP.computeRawAugObs2;
keyboard

SPPP.computeBRRS2;
SPPP.simulateBR2;
SPPP.computeARRS2;
SPPP.simulateAR;
SPPP.simulateFull;
end