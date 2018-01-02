function P = initPlanner(SPPP, RTTRS)
% initRTTFaSTrack(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

numVeh = length(SPPP.initStates);

%decrease target radius by tracking error bound
targetRsmall = SPPP.targetR - RTTRS.trackingRadius; 

%initializing dynamics for planner
pMax = RTTRS.dynSys.pMax;

P = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  P{i} = SPPPlanePlanner([SPPP.initStates{i}(1), SPPP.initStates{i}(2)], pMax(1), pMax(2));
  
  % Target set (for convenience)
  P{i}.target = shapeCylinder(SPPP.g2D, [SPPP.targetCenters{i}], ... %%%%%%%%%%%%% Can make this a box if needed
    SPPP.targetR);
  P{i}.targetsm = shapeCylinder(SPPP.g2D, [SPPP.targetCenters{i}], ...
    targetRsmall);
  P{i}.targetCenter = SPPP.targetCenters{i};
  P{i}.targetR = SPPP.targetR;
  P{i}.targetRsmall = targetRsmall;
 
end
end