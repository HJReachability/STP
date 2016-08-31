classdef SPPPlane < Plane
  % Specialized Plane class with SPP-related properties for the SPP project 
  
  properties
    vReserved
    wReserved
    
    BRS1
    BRS1_tau
    
    nomTraj
    nomTraj_tau
    
    obsForRTT
    obsForRTT_tau
    
    obsForIntr
    obsForIntr_tau
    
    FRS1
    FRS1_tau
    FRS1_g
    
    obs2D
    obs2D_tau
    
    target
    targetsm
    targetCenter
    targetR
    targetRsmall
    
    % Replanning
    replan = false % Whether replan is done
    tauBR
    tauAR
    tau
  end
  
  methods
    function obj = SPPPlane(varargin)
      % obj = SPPPlane(x, wMax, vrange, dMax)
      %   Simply call Plane constructor
      obj@Plane(varargin{:});
    end
  end
  
end

