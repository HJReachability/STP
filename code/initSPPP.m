function SPPP = initSPPP(simName, extraArgs)

switch simName
  case 'SF_dstb_11'
    %% Vehicle
    vehParams.vRangeA = [0.1 2.5];
    vehParams.wMaxA = 2;
    vehParams.dMaxA = [1.1 0];
    
    vehParams.vReserved = [1 -1.2];
    vehParams.wReserved = -0.8;
    
    %% RTT tracking radius
    trackingRadius = 3.5;
    
    %% Grid
    gridParams.min = [-50; -50; 0];
    gridParams.max = [500; 500; 2*pi];
    gridParams.min = [111; 111; 11];

    %% SPP Problem
    targetR = 10;
    tMin = -500;
    dt = 0.25;
    Rc = 1;
    
    %% Initial states and targets
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
    
    %% Obstacles
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

    %% Plot setup
    mapFile = 'map_earth.png';
    plotSPPP(mapFile, targetCentersSet, targetR, SPPP.g2D, staticObs, ...
      initState); 
    
    augStaticObs = addCRadius(SPPP.g2D, staticObs, trackingRadius);
    SPPP.augStaticObs = repmat(augStaticObs, ...
      [1 1 gridParams.N(3) length(SPPP.tau)]);    
    
    if isfield(extraArgs, 'RTTRS_file')
      load(RTTRS_file)
      SPPP.setRTTRS(RTTRS);
    end
    
    %% Initialize
    SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, ...
      gridParams);
  case 'SF_dstb_6'
    params.trackingRadius = 0.5;
    params.dMaxA = [1.1 0];
    
  
  case 'BA_dstb_11'
  case 'SF_intr_2'
  case 'SF_intr_3'
  otherwise
    error('Unknown simulation name!')
end