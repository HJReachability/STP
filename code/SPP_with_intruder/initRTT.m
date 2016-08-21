function Q = initRTT(SPPP, RTTRS)
% initRTT(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

% Reduce target by the size of the RTT tracking radius
targetRsmall = SPPP.targetR - RTTRS.trackingRadius;

numVeh = length(SPPP.initStates);
Q = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  Q{i} = Plane(SPPP.initStates{i}, ...
    RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);
  
  % Target set (for convenience)
  Q{i}.data.target = ...
    shapeCylinder(SPPP.g, 3, SPPP.targetCenters{i}, SPPP.targetR);
  Q{i}.data.targetsm = ...
    shapeCylinder(SPPP.g, 3, SPPP.targetCenters{i}, targetRsmall);
  Q{i}.data.targetCenter = SPPP.targetCenters{i};
  Q{i}.data.targetR = SPPP.targetR;
  Q{i}.data.targetRsmall = SPPP.targetRsmall;
  
  % Reserved control authorities
  Q{i}.data.vReserved = RTTRS.dynSys.vRangeB - RTTRS.dynSys.vRangeA;
  Q{i}.data.wReserved = RTTRS.dynSys.wMaxB - RTTRS.dynSys.wMaxA;
end
end