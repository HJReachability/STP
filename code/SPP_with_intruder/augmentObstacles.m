function vehicle = augmentObstacles(vehicle, schemeData, rawObs, debug)
% vehicle = computeBaseObs(vehicle, schemeData, resetR)
%     Computes the induced base obstacles by vehicle according to BRS1 and the
%     centralized controller scheme; populates the .baseObs and .baseObs_tau
%     fields of vehicle.data
%
% Inputs
%     vehicle:
%         vehicle object; should have the .data field populated with .BRS1 and
%         .BRS1_tau
%     schemeData:
%         parameters for the HJI PDE solver; should contain the fields .grid and
%         .dynSys
%     resetR:
%         minimum size for the reachable set during evolution; if the reachable
%         set goes below this size, the reachable set will be propagated
%         "maximally" until the size becomes large enough
%
% Output
%     vehicle:
%         updated vehicle object with .baseObs and .baseObs_tau populated in the
%         vehicle.data field

if nargin < 4
  debug = false;
end

if debug
  fprintf('Debugging %s...\n', mfilename)
  nomTraj_tau = 0:0.01:1;
else
  vehicle.data.cylObsBRS = zeros(size(rawObs.cylObsBRS));
  nomTraj_tau = vehicle.data.nomTraj_tau;
end

small = 1e-4;
for i = 1:length(nomTraj_tau)
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state
  
  % For the last tauIAT time steps, use the i-step BRS
  if nomTraj_tau(i) + max(rawObs.tauIAT) > max(nomTraj_tau)
    obsInd = find(rawObs.tauIAT > max(nomTraj_tau)-nomTraj_tau(i) - small & ...
      rawObs.tauIAT < max(nomTraj_tau)-nomTraj_tau(i) + small);
  else
    obsInd = length(rawObs.tauIAT);
  end
  
  if debug
    fprintf('i = %d; obsInd = %d\n', i, obsInd);
  else
    p = vehicle.data.nomTraj(1:2,i);
    t = vehicle.data.nomTraj(3,i);
    rawObsDatai = ...
      rotateData(schemeData.grid, rawObs.cylObsBRS(:,:,:,obsInd), t, [1 2], 3);
    rawObsDatai = shiftData(schemeData.grid, rawObsDatai, p, [1 2]);
    
    vehicle.data.cylObsBRS(:,:,:,i) = max(rawObsDatai, -vehicle.data.target);
  end
end
end