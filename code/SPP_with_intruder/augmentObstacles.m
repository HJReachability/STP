function vehicle = augmentObstacles(vehicle, schemeData, rawObsBRS, debug)

if nargin < 4
  debug = false;
end

if debug
  fprintf('Debugging %s...\n', mfilename)
  nomTraj_tau = 0:0.01:1;
else
  nomTraj_tau = vehicle.data.nomTraj_tau;
  vehicle.data.cylObsBRS_tau = vehicle.data.nomTraj_tau;
  vehicle.data.cylObsBRS = zeros([schemeData.grid.N' length(nomTraj_tau)]);
end

small = 1e-4;
for i = 1:length(nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(nomTraj_tau))
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state
  
  % For the last tauIAT time steps, use the i-step BRS
  if nomTraj_tau(i) + max(rawObsBRS.tauIAT) > max(nomTraj_tau)
    obsInd = find(rawObsBRS.tauIAT > max(nomTraj_tau)-nomTraj_tau(i)-small & ...
      rawObsBRS.tauIAT < max(nomTraj_tau)-nomTraj_tau(i)+small);
  else
    obsInd = length(rawObsBRS.tauIAT);
  end
  
  if debug
    fprintf('i = %d; obsInd = %d\n', i, obsInd);
  else
    p = vehicle.data.nomTraj(1:2,i);
    t = vehicle.data.nomTraj(3,i);
    rawObsDatai = rotateData( ...
      schemeData.grid, rawObsBRS.data(:,:,:,obsInd), t, [1 2], 3);
    rawObsDatai = shiftData(schemeData.grid, rawObsDatai, p, [1 2]);
    
    vehicle.data.cylObsBRS(:,:,:,i) = rawObsDatai;
    
    % Dealing with periodicity
    if t >= 0
      rawObsDatai = rotateData( ...
        schemeData.grid, rawObsBRS.data(:,:,:,obsInd), t-2*pi, [1 2], 3);
    else
      rawObsDatai = rotateData( ...
        schemeData.grid, rawObsBRS.data(:,:,:,obsInd), t+2*pi, [1 2], 3);      
    end
    vehicle.data.cylObsBRS(:,:,:,i) = ...
      min(vehicle.data.cylObsBRS(:,:,:,i), rawObsDatai);
    
    % Exclude target set
    vehicle.data.cylObsBRS(:,:,:,i) = ...
      max(vehicle.data.cylObsBRS(:,:,:,i), -vehicle.data.target);
  end
end
end