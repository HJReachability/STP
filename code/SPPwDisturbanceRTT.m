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

  numVeh = 20;
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
    target_ind = randi(4);
    targetCenters{i} = targetCentersSet{target_ind};
  end
  
  targetR = 20;
  
  % Vehicle parameters
  vehParams.vRangeA = [0.1 2.5];
  vehParams.wMaxA = 2;
  vehParams.dMaxA = [1.1 0]; % 11 m/s wind is "high wind" or "strong breeze"
  
  % Grid parameters
  gridParams.min = [-50; -50; 0];
  gridParams.max = [500; 500; 2*pi];
  gridParams.N = [125; 125; 15];  
  
  SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, gridParams);
  
  SPPP.tMin = -500;
  SPPP.dt = 2;
  SPPP.Rc = 1;
  SPPP.tau = SPPP.tMin:SPPP.dt:SPPP.tTarget;
  
  % Financial District
  Obs1 = shapeRectangleByCorners(SPPP.g, [300; 250; -inf], [350; 300; inf]);
  
  % Union Square
  Obs2 = shapeRectangleByCorners(SPPP.g, [-25; -30; -inf], [25; 30; inf]);
  Obs2 = rotateData(SPPP.g, Obs2, 7.5*pi/180, [1 2], []);
  Obs2 = shiftData(SPPP.g, Obs2, [325 185], [1 2]);
  Obs2b = shapeHyperplaneByPoints(SPPP.g, [170 0 0; 400 230 0; 160 0 pi], ...
    [0 500 0]);
  Obs2 = shapeDifference(Obs2, Obs2b);
  
  % City Hall
  Obs3 = shapeRectangleByCorners(SPPP.g, [-25; -5; -inf], [25; 5; inf]);
  Obs3 = rotateData(SPPP.g, Obs3, 7.5*pi/180, [1 2], []);
  Obs3 = shiftData(SPPP.g, Obs3, [170 65], [1 2]);
  
  staticObs = min(Obs1, Obs2);
  staticObs = min(staticObs, Obs3);
  SPPP.staticObs = repmat(staticObs, [1 1 1 length(SPPP.tau)]);
  
  % Plot setup
  mapFile = 'map_earth.png';
  [~, obs2D] = proj(SPPP.g, staticObs, [0 0 1]);
  plotSPPP(mapFile, targetCentersSet, targetR, SPPP.g2D, obs2D, initState);
  
  % RTT parameters
  vReserved = [1 -1.2];
  wReserved = -0.8;
  trackingRadius = 3.5;

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