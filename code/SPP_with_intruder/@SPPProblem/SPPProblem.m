classdef SPPProblem
  properties
    % Problem parameters
    initStates
    targetCenters
    
    targetR
    targetRsmall
    
    intrIS
    intrCtrl
    
    % SPP vehicle parameters
    vRangeA = [0.1 1];
    wMaxA = 1;
    dMaxA = [0.1 0.2];
    
    % Time and space
    tMin = -5    % Minimum time for the entire problem
    dt = 0.01    % Discretization
    tTarget = 0  % initial target time
    tIntr        % Intruder appearance time
    tReplan      % time at which replanning is done (intruder disappearance)
    g = createGrid([-1; -1; -3*pi/2], [1; 1; pi/2], [71; 71; 71], 3); % grid
    
    % Files to load
    RTTRS_filename % robust trajectory tracking reachable set
    CARS_filename  % collision avoidance reachable set
    rawObs_filename % raw obstacles file name
    
    BR_RS_filename % before replanning reachable sets
    AR_RS_filename % after replanning reachable sets
    
    BR_sim_filename % before replanning simulation file
    AR_sim_filename % after replanning simulation file
    resim_filename % resimulation file name
  end
  
  methods
    %% Contructor
    function obj = SPPProblem(initStates, targetCenters, targetR)
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
          [ 0.7; -0.3; 0]; ...
          [-0.7; -0.3; 0]};
      end
      
      if nargin < 3
        targetR = 0.15;
      end
      
      obj.initStates = initStates;
      obj.targetCenters = targetCenters;
      obj.targetR = targetR;
    end
  end
end