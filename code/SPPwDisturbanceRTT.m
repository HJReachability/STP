function SPPwDisturbanceRTT(SPPP)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 1
%   % for ACC and TCST paper
%   initStates = { ...
%     [-0.5; 0;  0]; ...
%     [ 0.5; 0; -pi]; ...
%     [-0.6; 0.6; -pi/4]; ...
%     [ 0.6; 0.6; -3*pi/4]};
%   
%   targetCenters = { ...
%     [ 0.7;  0.2; 0]; ...
%     [-0.7;  0.2; 0]; ...
%     [ 0.7; -0.7; 0]; ...
%     [-0.7; -0.7; 0]};

%   targetR = 0.1;
%   % Vehicle parameters for ACC and TCST papers
%   vehParams.vRangeA = [0.5 1];
%   vehParams.wMaxA = 1;
%   vehParams.dMaxA = [0.1 0.2];

  initStates = { ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4]; ...
    [400; 400;  5*pi/4] ...
    };
  
  targetCenters = { ...
    [100; 100]; ...
    [300; 100]; ...
    [100; 100]; ...
    [100; 300] ...
    };
  
  targetR = 20;
  
  % Vehicle parameters
  vehParams.vRangeA = [0.1 2.5];
  vehParams.wMaxA = 2;
  vehParams.dMaxA = [0.8 0]; % 15 m/s wind is "high wind"
  
  % Grid parameters
  gridParams.min = [0; 0; 0];
  gridParams.max = [500; 500; 2*pi];
  gridParams.N = [251; 251; 101];  
  
  SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, gridParams);
  
  SPPP.tMin = -300;
  SPPP.dt = 2;
  SPPP.Rc = 1;
  SPPP.tau = SPPP.tMin:SPPP.dt:SPPP.tTarget;
  staticObs = shapeRectangleByCorners(SPPP.g, [300; 300; -inf], ...
    [350; 350; inf]);
  SPPP.staticObs = repmat(staticObs, [1 1 1 length(SPPP.tau)]);
  
  % RTT parameters
  vReserved = [1 -1];
  wReserved = -0.8;
  trackingRadius = 10;

  fprintf('Enter any modifications to the SPPProblem...\n')
  keyboard
end

% % RTT parameters for ACC and TCST papers
% vReserved = [0.25 -0.25];
% wReserved = -0.4;
% trackingRadius = 0.075;

SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);
SPPP.computeNIRS;
SPPP.simulateNI;
end