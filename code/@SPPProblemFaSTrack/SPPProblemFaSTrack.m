classdef SPPProblemFaSTrack < handle
  properties
    % Problem parameters
    setupName
    initStates
    targetCenters
    
    targetR
    targetRsmall   %reduced target radius for RTT
    targetCentersSet
    
    intrIS
    intrCtrl
    tIAT
    
    % Dynamics
    uMin
    uMax
    aMin
    aMax
    dMin
    dMax
    pMin
    pMax
    
    % Planner and Tracker
    P
    Q
    
    % Intruder method 2
    max_num_affected_vehicles
    buffer_duration
    buffer_duration_ind
    remaining_duration_ind
    
    Rc = 0.1 % collision radius
    trackingRadius
    extraArgs
    
    % static obstacles
    mapFile
    staticObs
    augStaticObs
    
    % Time
    tMin = -5    % Minimum time for the entire problem
    dt = 0.01    % Discretization
    tTarget = 0  % initial target time
    tIntr        % Intruder appearance time
    tReplan      % time at which replanning is done (intruder disappearance)
    max_BRS_time % max amount of time BRS computation will run per vehicle
    
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
    RBR_filename          % "Relative buffer region" (min min on top of max min)
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
    function obj = SPPProblemFaSTrack(problem_name, extraArgs)
      if nargin < 2
        extraArgs = [];
      end
      
      setupName = problem_name;
      
      obj.folder = sprintf('%s_%f', mfilename, now);
      system(sprintf('mkdir %s', obj.folder));
      
      switch problem_name
        case 'buffer_illustration'
          obj.loadSetupFaSTrack(problem_name);
          
        case 'SF_dstb'
          extraArgs.number_of_vehicles = 50;
          extraArgs.dstb_or_intr = 'dstb';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
        case 'SF_dstb3'
          extraArgs.number_of_vehicles = 2;
          extraArgs.dstb_or_intr = 'dstb';
          %extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
        case 'room_dstb'
          extraArgs.number_of_vehicles = 1;
          extraArgs.dstb_or_intr = 'dstb';
          %extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('room', extraArgs);  
          
        case 'SF_intr_2'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 2;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion2_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;     %%%%%%%%%%%%%%%%%%%%Problem: this uses obj.RTT_tR, which 
                                           %is not defined anymore

        case 'buffer_region_steps'
          extraArgs.number_of_vehicles = 1;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 3;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion3_6.mat')
          
        case 'SF_intr_3'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
%           extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 3;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion3_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;  %%%%%%%%%%%%%%%%%%%%%
          
        case 'SF_intr_4'
          extraArgs.number_of_vehicles = 50;
          extraArgs.wind_speed = 6;
          extraArgs.separation_time = 45;
          extraArgs.dstb_or_intr = 'intr';
          extraArgs.ISTC_filename = 'SF_ISTC.mat';
          
          obj.loadSetupFaSTrack('SF', extraArgs);
          
          %% Intruder-related
          obj.max_num_affected_vehicles = 4;
          
          obj.add_data_file('RTTRS', 'RTTRS6.mat');
          obj.add_data_file('CARS', 'CARS6.mat');
          obj.add_data_file('bufferRegion', 'bufferRegion4_6.mat')
          obj.add_data_file('FRSBRS', 'FRSBRS6.mat')
          obj.augment_staticObs_intr2;       %%%%%%%%%%%%%%%%%%%  
          
        case 'Bay_Area'
          extraArgs.number_of_vehicles = 200;
          extraArgs.wind_speed = 11;
          extraArgs.separation_time = 10;
          
          obj.loadSetupFaSTrack('Bay_Area', extraArgs);
          
        case 'TCST_dstb'
        case 'TCST_intr'
        otherwise
          error('Unknown simulation name!')
      end
      
      SPPP = obj;  
      save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
    end
  end  
end

