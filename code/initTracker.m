function Q = initTracker(SPPP, RTTRS)
% initRTTFaSTrack(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

numVeh = length(SPPP.initStates);

%decrease target radius by tracking error bound
targetRsmall = SPPP.targetR - RTTRS.trackingRadius; 

%initializing dynamics for tracker
uMax = RTTRS.dynSys.uMax;
aMax = RTTRS.dynSys.aMax;
dMax = RTTRS.dynSys.dMax;
dMin = RTTRS.dynSys.dMin;
pMin = RTTRS.dynSys.pMin;
pMax = RTTRS.dynSys.pMax;


Q = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  Q{i} = SPPPlaneTracker(SPPP.initStates{i}, -uMax, uMax, pMin, pMax, ...
  dMin, dMax, -aMax, aMax, [1 2 3 4]);
  
  % Target set (for convenience)
  Q{i}.target = shapeCylinder(SPPP.g2D, [SPPP.targetCenters{i}; 0; 0], ... 
    SPPP.targetR);
  Q{i}.targetsm = shapeCylinder(SPPP.g2D, [SPPP.targetCenters{i}; 0; 0], ...
    targetRsmall);
  Q{i}.targetCenter = [SPPP.targetCenters{i}; 0; 0];
  Q{i}.targetR = SPPP.targetR;
  Q{i}.targetRsmall = targetRsmall;
 
end
end