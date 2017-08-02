function Q = initRTTFaSTrack(SPPP, RTTRS)
% initRTTFaSTrack(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

numVeh = length(SPPP.initStates);

%decrease target radius by tracking error bound
targetRsmall = SPPP.targetR - RTTRS.trackingRadius; 

uMax = RTTRS.dynSys.uMax;
aMax = RTTRS.dynSys.aMax;
dMax = RTTRS.dynSys.dMax;

Q = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  Q{i} = SPPPlaneFaSTrack(SPPP.initStates{i}, uMax, aMax, dMax);
  
  % Target set (for convenience)
  Q{i}.target = shapeCylinder(SPPP.g, 3, [SPPP.targetCenters{i}; 0], ...
    SPPP.targetR);
  Q{i}.targetsm = shapeCylinder(SPPP.g, 3, [SPPP.targetCenters{i}; 0], ...
    targetRsmall);
  Q{i}.targetCenter = SPPP.targetCenters{i};
  Q{i}.targetR = SPPP.targetR;
  Q{i}.targetRsmall = targetRsmall;
  
  Q{i}.pMin = RTTRS.dynSys.pMin;
  Q{i}.pMax = RTTRS.dynSys.pMax;
 
end
end