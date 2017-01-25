function loadSetup(obj, setup_name, extraArgs)

switch setup_name
  case 'SF'
    if nargin < 3
      error('Must specify extraArgs for ''SF'' setup!')
    end
    
    %% Vehicle
    obj.vRangeA = [0.1 2.5];
    obj.wMaxA = 2;
    
    %% RTT
    obj.vReserved = [1 -1.2];
    obj.wReserved = -0.8;
    
    %% Initial states
    if isfield(extraArgs, 'number_of_vehicles')
      numVeh = extraArgs.number_of_vehicles;
    else
      error('Must specify the property ''number_of_vehicles''!')
    end
    
    %% Sampling and collision radius
    obj.dt = 0.5;
    obj.Rc = 1;
    
    %% Initial targets
    obj.targetR = 10;
    
    if isfield(extraArgs, 'ISTC_filename')
      fprintf('Loading initial states and target centers...\n')
      load(extraArgs.ISTC_filename)
      obj.initStates = IS;
      obj.targetCenters = TC;
    else
      [obj.initStates, obj.targetCenters] = ...
        gen_targets_initStates(setup_name, numVeh);
    end

    %% Grid
    % Defaults
    obj.gMin = [0 0 0];
    obj.gMax = [500 500 2*pi];
    obj.gN = [101 101 15];
    
    wind_prompt = 'Please enter wind speed in m/s (6 or 11 only): ';
    wind_speed = input(wind_prompt);
    while ~any(wind_speed == [6 11])
      wind_speed = input(wind_prompt);
    end
    
    % Custom modifications
    if strcmp(extraArgs.dstb_or_intr, 'dstb') && wind_speed == 6
      obj.gN = [201 201 15];
    elseif strcmp(extraArgs.dstb_or_intr, 'intr')
      obj.gMin = [-50 -50 0];
      obj.gMax = [550 550 2*pi];
      obj.gN = [125 125 15];
    end
    
    obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
    obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
    
    %% Obstacles
    temp_g2D = createGrid([-35 -35], [35 35], [101 101]);
    
    % Financial District
    Obs1 = shapeRectangleByCorners(obj.g2D, [300; 250], [350; 300]);
    
    % Union Square
    Obs2 = shapeRectangleByCorners(temp_g2D, [-25; -30], [25; 30]);
    Obs2_rot = rotateData(temp_g2D, Obs2, 7.5*pi/180, [1 2], []);
    Obs2_gShift = shiftGrid(temp_g2D, [325 185]);
    Obs2 = migrateGrid(Obs2_gShift, Obs2_rot, obj.g2D);
    Obs2b = shapeHyperplaneByPoints(obj.g2D, [170 0; 400 230], ...
      [0 500]);
    Obs2 = shapeDifference(Obs2, Obs2b);
    
    % City Hall
    Obs3 = shapeRectangleByCorners(temp_g2D, [-25; -5], [25; 5]);
    Obs3_rot = rotateData(temp_g2D, Obs3, 7.5*pi/180, [1 2], []);
    Obs3_gShift = shiftGrid(temp_g2D, [170 65]);
    Obs3 = migrateGrid(Obs3_gShift, Obs3_rot, obj.g2D);
    
    obj.staticObs = min(Obs1, Obs2);
    obj.staticObs = min(obj.staticObs, Obs3);
    
    obj.mapFile = 'map_streets.png';
    
    %% Wind speed
    switch wind_speed
      case 6
        obj.dMaxA = [0.6 0];
        obj.RTT_tR = 0.5;
      case 11
        obj.dMaxA = [1.1 0];
        obj.RTT_tR = 3.5;
    end
    
    
    %% Target separation time
    separation_time = input('Please enter separation time in seconds: ');
    
    % Scheduled times of arrival
    obj.tTarget = zeros(numVeh, 1);
    for i = 1:numVeh
      obj.tTarget(i) = -separation_time*(i-1);
    end
    
    % Adjust global time horizon
    obj.tMin = -500 - numVeh*separation_time;
    obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
    
    %% Augment static obstacles
    if isfield(extraArgs, 'dstb_or_intr')
      switch extraArgs.dstb_or_intr
        case 'dstb'
          augStaticObs = addCRadius(obj.g2D, obj.staticObs, obj.RTT_tR);

          % Boundary obstacle
          obs_bdry = -shapeRectangleByCorners(obj.g2D, obj.g2D.min + [5;5], ...
            obj.g2D.max - [5;5]);
          obj.augStaticObs = min(augStaticObs, obs_bdry);

        case 'intr'
          obj.tIAT = 10;
          warning(['Static obstacles will be augmented after computing ' ...
            'bufferRegion and FRSBRS'])
          
        otherwise
          error('The property ''dstb_or_intr'' must be ''dstb'' or ''intr''!')
      end
    else
      error('Must specify property ''dstb_or_intr'' in extraArgs!')
    end
    
    obj.plotSetup(setup_name);
    
  case 'Bay_Area'
    %% Vehicle
    obj.vRangeA = [0.1 2.5];
    obj.wMaxA = 2;
    
    %% RTT
    obj.vReserved = [1 -1.2];
    obj.wReserved = -0.8;
    
    %% Initial states and targets
    if isfield(extraArgs, 'number_of_vehicles')
      numVeh = extraArgs.number_of_vehicles;
    else
      error('Must specify the property ''number_of_vehicles''!')
    end
    
    IC_and_target_centers = {[350; 180]; ...
      [1190; 380]; ...
      [1150; 1070]; ...
      [650; 1620]};
    
    obj.targetR = 10;
    obj.initStates = cell(numVeh, 1);
    obj.targetCenters = cell(numVeh,1);
    numTar = length(IC_and_target_centers);
    tarCount = zeros(numTar);
    for i = 1:numVeh
      % Randomize target and IC
      IC_index = randi(numTar);
      target_index = randi(numTar);
      while target_index == IC_index
        target_index = randi(numTar);
      end
      
      tarCount(IC_index, target_index) = tarCount(IC_index, target_index) + 1;
      
      % Assign IC and target
      IC_pos = IC_and_target_centers{IC_index};
      target_pos = IC_and_target_centers{target_index};
      
      delta = target_pos - IC_pos;
      IC_angle = wrapTo2Pi(atan2(delta(2), delta(1)));
      
      obj.initStates{i} = [IC_pos; IC_angle];
      obj.targetCenters{i} = [target_pos; 0];
    end
    
    disp('tarCount = ')
    disp(tarCount)
    
    %% Sampling and collision radius
    obj.dt = 0.5;
    obj.Rc = 1;

    %% Grid
    obj.gMin = [250 80 0];
    obj.gMax = [1290 1720 2*pi];
    obj.gN = [209 329 15];

    obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
    obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));    
    
    obj.mapFile = 'bay_area_streets.png';
    
    %% Wind speed
    obj.dMaxA = [1.1 0];
    obj.RTT_tR = 3.5;
    
    %% Target separation time
    % Scheduled times of arrival
    separation_time = 45;
    obj.tTarget = zeros(numVeh, 1);
    for i = 1:numVeh
      obj.tTarget(i) = -separation_time*(i-1);
    end
    
    % Adjust global time horizon
    obj.tMin = -1500 - numVeh*separation_time;
    obj.tau = obj.tMin:obj.dt:max(obj.tTarget);
    
    obj.plotSetup(setup_name);
    
  otherwise
    error('Unknown setup_name!')
end

end