classdef SPPPlane < Plane
  % Specialized Plane class for the SPP project
  
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
    
    obs2D
    obs2D_tau
    
    target
    targetsm
    targetCenter
  end
  
  methods
    function obj = SPPPlane(varargin)
      % Simply call Plane constructor
      obj@Plane(varargin{:});
    end
  end
  
end

