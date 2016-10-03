classdef SPPProblem < handle
  properties
    % Problem parameters
    initStates
    targetCenters
    
    targetR
    targetRsmall
    
    intrIS
    intrCtrl
    
    Rc = 0.1 % collision radius
    
    % SPP vehicle parameters
    vRangeA = [0.1 1];
%     vRangeA = [0.5 1];
    wMaxA = 1;
    dMaxA = [0.1 0.2];
    
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
    tau          % Time vector for entire simulation
    
    % File to store this SPPProblem instance
    this_filename
    
    % Files to load
    RTTRS_filename % robust trajectory tracking reachable set
    CARS_filename  % collision avoidance reachable set
    rawAugObs_filename % raw augmented obstacles file name
    
    NI_RS_filename % no intruder reachable sets
    NI_RS_filename_small
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
    function obj = SPPProblem(initStates, targetCenters, targetR, vehParams, ...
        grid_params)
      if nargin < 1
        initStates = { ...
          [-0.6; 0.2;  0]; ...
          [ 0.6; 0.2; -pi]; ...
          [-0.5; 0.9; -pi/4]; ...
          [ 0.5; 0.9; -3*pi/4]};
      end
      
      if nargin < 2
        targetCenters = { ...
          [ 0.7;  0.7; 0]; ...
          [-0.7;  0.7; 0]; ...
          [ 0.7; -0.7; 0]; ...
          [-0.7; -0.7; 0]};
      end
      
      if nargin < 3
%         targetR = 0.15;
        targetR = 0.1;
      end
     
      if nargin < 4
        vehParams.vRangeA = [0.5 1];
        vehParams.wMaxA = 1;
        vehParams.dMaxA = [0.1 0.2];        
      end
      
      if nargin < 5
        grid_params.min = [-1; -1; -3*pi/2];
        grid_params.max = [1; 1; pi/2];
        grid_params.N = [95; 95; 95];
      end
      
      obj.initStates = initStates;
      obj.targetCenters = targetCenters;
      obj.targetR = targetR;
      
      obj.vRangeA = vehParams.vRangeA;
      obj.wMaxA = vehParams.wMaxA;
      obj.dMaxA = vehParams.dMaxA;
      
      obj.gMin = grid_params.min;
      obj.gMax = grid_params.max;
      obj.gN = grid_params.N;
      
      obj.g = createGrid(obj.gMin, obj.gMax, obj.gN, 3);
      obj.g2D = createGrid(obj.gMin(1:2), obj.gMax(1:2), obj.gN(1:2));
      
      obj.this_filename = sprintf('%s_%f.mat', mfilename, now);
    end
  end
end