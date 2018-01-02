classdef SPPPlaneTracker < P4D_Q2D_Rel
    %UNTITLED8 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    BRS1
    BRS1_tau    
    
    nomTraj
    nomTraj_tau     

    nomTraj_AR % Nominal trajectory after replanning
    nomTraj_AR_tau
    
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
    function obj = SPPPlaneTracker(varargin)
      % obj = SPPPlane(x, wMax, vrange, dMax)
      %   Simply call Plane constructor
      obj@P4D_Q2D_Rel(varargin{:});
    end
  end
    
end

