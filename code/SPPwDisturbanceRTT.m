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

  numVeh = 50;
  initStates = cell(numVeh, 1);
  initState = [475; 200; 220*pi/180];
  for i = 1:numVeh
    initStates{i} = initState;
  end
  
  targetCentersSet = { ...
    [300; 400]; ...
    [50; 175]; ...
    [75; 25]; ...
    [450; 25] ...
    };
  
  targetCenters = cell(numVeh,1);
  for i = 1:numVeh
    target_ind = randi(length(targetCentersSet));
    targetCenters{i} = targetCentersSet{target_ind};
  end
  
  targetR = 10;
  
  % Vehicle parameters
  vehParams.vRangeA = [0.1 2.5];
  vehParams.wMaxA = 2;
%   vehParams.dMaxA = [1.1 0]; % 11 m/s wind is "high wind" or "strong breeze"
  vehParams.dMaxA = [0.6 0]; % 6 m/s wind is "moderate breeze"

  % RTT parameters
  vReserved = [1 -1.2];
  wReserved = -0.8;
%   trackingRadius = 3.5;
  trackingRadius = 0.5;
  
  % Grid parameters
  gridParams.min = [-10; -10; 0];
  gridParams.max = [500; 500; 2*pi];
  gridParams.N = [101; 101; 11];  
  
  SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, gridParams);
  
  SPPP.tMin = -500;
  SPPP.dt = 0.25;
  SPPP.Rc = 1;
  SPPP.tau = SPPP.tMin:SPPP.dt:SPPP.tTarget;
  
  % Financial District
  Obs1 = shapeRectangleByCorners(SPPP.g2D, [300; 250], [350; 300]);
  
  % Union Square
  Obs2 = shapeRectangleByCorners(SPPP.g2D, [-25; -30], [25; 30]);
  Obs2 = rotateData(SPPP.g2D, Obs2, 7.5*pi/180, [1 2], []);
  Obs2 = shiftData(SPPP.g2D, Obs2, [325 185], [1 2]);
  Obs2b = shapeHyperplaneByPoints(SPPP.g2D, [170 0; 400 230], ...
    [0 500]);
  Obs2 = shapeDifference(Obs2, Obs2b);
  
  % City Hall
  Obs3 = shapeRectangleByCorners(SPPP.g2D, [-25; -5], [25; 5]);
  Obs3 = rotateData(SPPP.g2D, Obs3, 7.5*pi/180, [1 2], []);
  Obs3 = shiftData(SPPP.g2D, Obs3, [170 65], [1 2]);
  
  % Boundary
  Obs4 = -shapeRectangleByCorners(SPPP.g2D, SPPP.g2D.min+1, SPPP.g2D.max-1);
  
  staticObs = min(Obs1, Obs2);
  staticObs = min(staticObs, Obs3);
  staticObs = min(staticObs, Obs4);
  SPPP.staticObs = staticObs;
  
  % Plot setup
  mapFile = 'map_earth.png';
  plotSPPP(mapFile, targetCentersSet, targetR, SPPP.g2D, staticObs, initState);
  
  augStaticObs = addCRadius(SPPP.g2D, staticObs, trackingRadius);
  SPPP.augStaticObs = repmat(augStaticObs, ...
    [1 1 gridParams.N(3) length(SPPP.tau)]);
  
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