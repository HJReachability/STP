classdef SPPProblem < handle
  properties
    % Problem parameters
    initStates
    targetCenters
    
    targetR
    targetRsmall
    
    intrIS
    intrCtrl
    tIAT
    
    % Intruder method 2
    max_num_affected_vehicles
    buffer_duration
    buffer_duration_ind
    remaining_duration_ind
    
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
    max_BRS_time
    
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
    BR_RS_chkpt_filename
    AR_RS_filename % after replanning reachable sets
    AR_RS_chkpt_filename
    
    BR_sim_filename % before replanning simulation file (simulation results)
    full_sim_filename % full simulation file (simulation results)
  end
  
  methods
    %% Contructor
    function obj = SPPProblem(problem_name, extraArgs)
      if nargin < 2
        extraArgs = [];
      end
      
      obj.folder = sprintf('%s_%f', mfilename, now);
      system(sprintf('mkdir %s', obj.folder));
      
      switch problem_name
        case 'buffer_illustration'
          obj.loadSetup(problem_name);
          
        case 'SF_dstb'
          extraArgs.number_of_vehicles = 50;
          extraArgs.dstb_or_intr = 'dstb';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetup('SF', extraArgs);
          
        case 'SF_intr_2'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetup('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 2;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion2_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;          
          
        case 'SF_intr_3'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetup('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 3;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion3_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;
          
        case 'SF_intr_4'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetup('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 4;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion4_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;          
          
        case 'Bay_Area'
          extraArgs.number_of_vehicles = 200;
          extraArgs.wind_speed = 11;
          extraArgs.separation_time = 10;
          
          obj.loadSetup('Bay_Area', extraArgs);
          
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
      
      SPPP = obj;
      save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
    end
  end
end