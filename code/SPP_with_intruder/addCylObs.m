function vehicle = addCylObs(vehicle, schemeData, rawCylObs, debug)

if nargin < 4
  debug = false;
end

if debug
  fprintf('Debugging %s...\n', mfilename)
  nomTraj_tau = 0:0.01:1;
else
  vehicle.data.cylObs3D = zeros(size(rawCylObs.data));
  nomTraj_tau = vehicle.data.nomTraj_tau;
  vehicle.data.cylObs3D_tau = vehicle.data.nomTraj_tau;
end

small = 1e-4;
for i = 1:length(nomTraj_tau)
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state
  
  % For the first tauIAT time steps, use the i-step FRS projection
  if nomTraj_tau(i)-min(nomTraj_tau) < max(rawCylObs.tauIAT)
    obsInd = find(rawCylObs.tauIAT > nomTraj_tau(i)-min(nomTraj_tau)-small & ...
      rawCylObs.tauIAT < nomTraj_tau(i)-min(nomTraj_tau)+small);
  else
    obsInd = length(rawCylObs.tauIAT);
  end
  
  if debug
    fprintf('i = %d; obsInd = %d\n', i, obsInd);
  else
    p = vehicle.data.nomTraj(1:2,i);
    t = vehicle.data.nomTraj(3,i);
    rawObsDatai = ...
      rotateData(schemeData.grid, rawCylObs.data(:,:,:,obsInd), t, [1 2], 3);
    rawObsDatai = shiftData(schemeData.grid, rawObsDatai, p, [1 2]);
    
    vehicle.data.cylObsBRS(:,:,:,i) = max(rawObsDatai, -vehicle.data.target);
  end
end
end