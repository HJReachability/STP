function Q = initRTT(SPPP, RTTRS)
% initRTT(initStates, targetCenters, targetR, RTTRS)
%     Initializes Plane objects for the RTT method SPP problem

numVeh = length(SPPP.initStates);
targetRsmall = SPPP.targetR - RTTRS.trackingRadius;
wMax = RTTRS.dynSys.wMaxA;
vrange = RTTRS.dynSys.vRangeA;
dMax = RTTRS.dynSys.dMaxA;

Q = cell(numVeh, 1);
for i = 1:numVeh
  % Initial state and parameters
  Q{i} = SPPPlane(SPPP.initStates{i}, wMax, vrange, dMax);
  
  % Target set (for convenience)
  Q{i}.target = shapeCylinder(SPPP.g, 3, SPPP.targetCenters{i}, SPPP.targetR);
  Q{i}.targetsm = shapeCylinder(SPPP.g, 3, SPPP.targetCenters{i}, targetRsmall);
  Q{i}.targetCenter = SPPP.targetCenters{i};
  Q{i}.targetR = SPPP.targetR;
  Q{i}.targetRsmall = targetRsmall;
  
%   % Reserved control authorities
  Q{i}.vReserved = RTTRS.dynSys.vRangeB - vrange;
  Q{i}.wReserved = RTTRS.dynSys.wMaxB - wMax;
end
end