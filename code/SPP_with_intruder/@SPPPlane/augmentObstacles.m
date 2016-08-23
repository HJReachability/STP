function augmentObstacles(obj, g, rawObsBRS)
% vehicle = augmentObstacles(vehicle, schemeData, rawObsBRS, debug)
%     Computes the FRS + CR + BRS-augmented obstacles assuming the RTT method
%     for SPP with intruders

obj.obsForIntr_tau = obj.nomTraj_tau;
obj.obsForIntr = zeros([g.N' length(obj.nomTraj_tau)]);

small = 1e-4;
for i = 1:length(obj.nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(obj.nomTraj_tau))
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state
  % For the last tauIAT time steps, use the i-step BRS
  if obj.nomTraj_tau(i) + max(rawObsBRS.tauIAT) > max(obj.nomTraj_tau)
    obsInd = find( ...
      rawObsBRS.tauIAT > max(obj.nomTraj_tau)-obj.nomTraj_tau(i)-small & ...
      rawObsBRS.tauIAT < max(obj.nomTraj_tau)-obj.nomTraj_tau(i)+small);
  else
    obsInd = length(rawObsBRS.tauIAT);
  end
  
  % Rotate and shift raw obstacles
  p = obj.nomTraj(1:2,i);
  t = obj.nomTraj(3,i);
  rawObsDatai = rotateData(g, rawObsBRS.data(:,:,:,obsInd), t, [1 2], 3);
  rawObsDatai = shiftData(g, rawObsDatai, p, [1 2]);
  
  obj.data.obsForIntr(:,:,:,i) = rawObsDatai;
  
  % Dealing with periodicity
  if t >= 0
    rawObsDatai = rotateData(g, rawObsBRS.data(:,:,:,obsInd), t-2*pi, [1 2], 3);
  else
    rawObsDatai = rotateData(g, rawObsBRS.data(:,:,:,obsInd), t+2*pi, [1 2], 3);
  end
  rawObsDatai = shiftData(g, rawObsDatai, p, [1 2]);
  
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsDatai);
  
  % Exclude target set
  obj.obsForIntr(:,:,:,i) = max(obj.obsForIntr(:,:,:,i), -obj.target);
end
end