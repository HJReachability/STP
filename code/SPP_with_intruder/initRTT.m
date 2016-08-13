function Q = initRTT(initStates, targetCenters, targetR, RTTRS, schemeData)
% initRTT(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

% Reduce target by the size of the RTT tracking radius
targetRsmall = targetR - RTTRS.trackingRadius;

numVeh = length(initStates);
Q = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  Q{i} = Plane(initStates{i}, ...
    RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);
  
  % Target set
  Q{i}.data.target = ...
    shapeCylinder(schemeData.grid, 3, targetCenters{i}, targetR);
  Q{i}.data.targetsm = ...
    shapeCylinder(schemeData.grid, 3, targetCenters{i}, targetRsmall);
  Q{i}.data.targetCenter = targetCenters{i};
  Q{i}.data.targetR = targetR;
  Q{i}.data.targetRsmall = targetRsmall;
  
  % Reserved control authorities
  Q{i}.data.vReserved = RTTRS.dynSys.vRangeB - RTTRS.dynSys.vRangeA;
  Q{i}.data.wReserved = RTTRS.dynSys.wMaxB - RTTRS.dynSys.wMaxA;
end
end