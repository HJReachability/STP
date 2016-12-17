classdef SPPProblem < handle
  properties
    % Problem parameters
    initStates
    targetCenters
    
    targetR
    targetRsmall
    
    intrIS
    intrCtrl
    max_num_affected_vehicles
    
    Rc = 0.1 % collision radius
    
    % static obstacles
    mapFile
    staticObs
    augStaticObs
    
    % SPP vehicle parameters
    vRangeA;
    wMaxA;
    dMaxA;
    
    % Robust trajectory tracking
    vReserved
    wReserved
    RTT_tR
    
    % Time
    tMin = -5    % Minimum time for the entire problem
    dt = 0.01    % Discretization
    tTarget = 0  % initial target time
    tIntr        % Intruder appearance time
    tReplan      % time at which replanning is done (intruder disappearance)
    
    % Space
    gMin
    gMax
    gN
    g
    g2D
    
    tauBR        % Time vector before replanning
    tauAR        % Time vector after replanning
    tauSim       % Time vector for entire simulation
    tau          % Global time (absolute time vector)
    
    % File to store this SPPProblem instance
    folder
    
    % Files to load
    RTTRS_filename % robust trajectory tracking reachable set
    CARS_filename  % collision avoidance reachable set
    
    minMinBRS_filename    % minMinBRS file (for computing buffer region)
    bufferRegion_filename % Buffer region
    FRSBRS_filename       % FRSs and BRSs of FRSs
    rawAugObs_filename    % raw augmented obstacles file name
    
    NI_RS_chkpt_filename % no intruder reachable sets
    NI_RS_filename
    NI_sim_filename
    
    BR_RS_filename % before replanning reachable sets
    BR_RS_filename_small
    AR_RS_filename % after replanning reachable sets
    AR_RS_filename_small
    
    BR_sim_filename % before replanning simulation file (simulation results)
    full_sim_filename % full simulation file (simulation results)
  end
  
  methods
    %% Contructor
    function obj = SPPProblem(problem_name, extraArgs)
      if nargin < 2
        extraArgs = [];
      end
      
      switch problem_name
        case 'SF_dstb_11'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [1.1 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 3.5;
          
          %% Grid
          obj.gMin = [-46.45 -46.45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [101 101 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Time
          obj.tMin = -500;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:obj.tTarget;
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
          end
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'SF_dstb_6'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [0.6 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 0.5;
          
          %% Grid
          obj.gMin = [-45 -45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [201 201 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Time
          obj.tMin = -500;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:obj.tTarget;
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
          end
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'SF_dstb_6_2sSep'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [0.6 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 0.5;
          
          %% Grid
          obj.gMin = [-45 -45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [201 201 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Time
          obj.tMin = -600;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:obj.tTarget;
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          obj.tTarget = zeros(numVeh, 1);
          for i = 1:numVeh
            obj.initStates{i} = initState;
            obj.tTarget(i) = -2*i;
          end
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'SF_dstb_11_2sSep'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [1.1 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 3.5;
          
          %% Grid
          obj.gMin = [-46.45 -46.45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [101 101 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          obj.tTarget = zeros(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
            obj.tTarget(i) = -2*i;
          end
          
          %% Time
          obj.tMin = -450;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'SF_dstb_11_4sSep'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [1.1 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 3.5;
          
          %% Grid
          obj.gMin = [-46.45 -46.45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [101 101 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          obj.tTarget = zeros(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
            obj.tTarget(i) = -4*i;
          end
          
          %% Time
          obj.tMin = -700;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'SF_dstb_11_6sSep'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [1.1 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 3.5;
          
          %% Grid
          obj.gMin = [-46.45 -46.45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [101 101 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          obj.tTarget = zeros(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
            obj.tTarget(i) = -6*i;
          end
          
          %% Time
          obj.tMin = -800;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
        case 'BA_dstb_11'
        case 'SF_intr_2'
          %% Vehicle
          obj.vRangeA = [0.1 2.5];
          obj.wMaxA = 2;
          obj.dMaxA = [0.6 0];
          
          %% RTT
          obj.vReserved = [1 -1.2];
          obj.wReserved = -0.8;
          obj.RTT_tR = 0.5;
          
          %% Grid
          obj.gMin = [-46.45 -46.45 0];
          obj.gMax = [500 500 2*pi];
          obj.gN = [101 101 15];
          
          obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
          obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
          
          %% Initial states
          numVeh = 50;
          obj.initStates = cell(numVeh, 1);
          obj.tTarget = zeros(numVeh, 1);
          initState = [475; 200; 220*pi/180];
          for i = 1:numVeh
            obj.initStates{i} = initState;
            obj.tTarget(i) = -4*i;
          end
          
          %% Time
          obj.tMin = -1000;
          obj.dt = 0.5;
          obj.Rc = 1;
          obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
          
          %% Initial targets
          obj.targetR = 10;
          
          targetCentersSet = { ...
            [300; 400]; ...
            [50; 175]; ...
            [75; 25]; ...
            [450; 25] ...
            };
          
          obj.targetCenters = cell(numVeh,1);
          for i = 1:numVeh
            target_ind = randi(length(targetCentersSet));
            obj.targetCenters{i} = targetCentersSet{target_ind};
          end
          
          %% Obstacles
          % Financial District
          Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
          
          % Union Square
          Obs2 = shapeRectangleByCorners(obj.g2D, [-25; -30], [25; 30]);
          Obs2 = rotateData(obj.g2D, Obs2, 7.5*pi/180, [1 2], []);
          Obs2 = shiftData(obj.g2D, Obs2, [325 185], [1 2]);
          Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
            [0 500]);
          Obs2 = shapeDifference(Obs2, Obs2b);
          
          % City Hall
          Obs3 = shapeRectangleByCorners(obj.g2D, [-25; -5], [25; 5]);
          Obs3 = rotateData(obj.g2D, Obs3, 7.5*pi/180, [1 2], []);
          Obs3 = shiftData(obj.g2D, Obs3, [170 65], [1 2]);
          
          % Boundary
          Obs4 = -shapeRectangleByCorners(obj.g2D, obj.g2D.min+5, obj.g2D.max-5);
          
          obj.staticObs = min(Obs1, Obs2);
          obj.staticObs = min(obj.staticObs, Obs3);
          obj.staticObs = min(obj.staticObs, Obs4);
          
          obj.mapFile = 'map_streets.png';
          
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);
          obj.augStaticObs = repmat(augStaticObs, ...
            [1 1 obj.gN(3) length(obj.tau)]);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 2;
          
        case 'SF_intr_3'
        case 'TCST_dstb'
        case 'TCST_intr'
        otherwise
          error('Unknown simulation name!')
      end
      
      %       if nargin < 1
      %         initStates = { ...
      %           [-0.6; 0.2;  0]; ...
      %           [ 0.6; 0.2; -pi]; ...
      %           [-0.5; 0.9; -pi/4]; ...
      %           [ 0.5; 0.9; -3*pi/4]};
      %       end
      %
      %       if nargin < 2
      %         targetCenters = { ...
      %           [ 0.7;  0.7; 0]; ...
      %           [-0.7;  0.7; 0]; ...
      %           [ 0.7; -0.7; 0]; ...
      %           [-0.7; -0.7; 0]};
      %       end
      %
      %       if nargin < 3
      %         %         targetR = 0.15;
      %         targetR = 0.1;
      %       end
      %
      %       if nargin < 4
      %         vehParams.vRangeA = [0.5 1];
      %         vehParams.wMaxA = 1;
      %         vehParams.dMaxA = [0.1 0.2];
      %       end
      %
      %       if nargin < 5
      %         grid_params.min = [-1; -1; -3*pi/2];
      %         grid_params.max = [1; 1; pi/2];
      %         grid_params.N = [95; 95; 95];
      %       end
      %
      %       obj.initStates = initStates;
      %       obj.targetCenters = targetCenters;
      %       obj.targetR = targetR;
      %
      %       obj.vRangeA = vehParams.vRangeA;
      %       obj.wMaxA = vehParams.wMaxA;
      %       obj.dMaxA = vehParams.dMaxA;
      %
      %       obj.gMin = grid_params.min;
      %       obj.gMax = grid_params.max;
      %       obj.gN = grid_params.N;
      %
      %       obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
      %       obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
      %
      %       obj.tau = obj.tMin:obj.dt:obj.tTarget;
      %       obj.staticObs = inf(obj.gN');
      
      obj.folder = sprintf('%s_%f', mfilename, now);
      
      SPPP = obj;
      system(sprintf('mkdir %s', obj.folder));
      save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
      
      obj.plotSetup();
      
      if isfield(extraArgs, 'RTTRS_filename')
        load(extraArgs.RTTRS_filename)
        obj.setRTTRS(RTTRS);
      end
      
      if isfield(extraArgs, 'CARS_filename')
        load(extraArgs.CARS_filename)
        obj.setCARS(CARS);
      end
    end
  end
end