function computeObsForIntr(obj, g, CARS, rawAugObs)
% computeAugObs(obj, g, CARS, rawAugObs)
%     Computes the FRS + CR + BRS-augmented obstacles assuming the RTT method
%     for SPP with intruders

% Migrate the raw augmented obstacles
rawAugObs3D =  migrateGrid(rawAugObs.g, rawAugObs.data, g);

obj.obsForIntr_tau = obj.nomTraj_tau;
obj.obsForIntr = zeros([g.N' length(obj.nomTraj_tau)]);

small = 1e-4;
for i = 1:length(obj.nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(obj.nomTraj_tau))
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state
  
  % For the last tauIAT time steps, use the i-step BRS
  if obj.nomTraj_tau(i) + max(CARS.tau) > max(obj.nomTraj_tau)
    obsInd = find(CARS.tau > max(obj.nomTraj_tau)-obj.nomTraj_tau(i)-small & ...
      CARS.tau < max(obj.nomTraj_tau)-obj.nomTraj_tau(i)+small);
  else
    obsInd = length(CARS.tau);
  end
  
  % Rotate and shift raw obstacles
  p = obj.nomTraj(1:2,i);
  t = obj.nomTraj(3,i);
  rawObsDatai = rotateData(g, rawAugObs3D(:,:,:,obsInd), t, [1 2], 3);
  rawObsDatai = shiftData(g, rawObsDatai, p, [1 2]);
  
  obj.obsForIntr(:,:,:,i) = rawObsDatai;
  
  % Dealing with periodicity
  if t >= 0
    rawObsDatai = rotateData(g, rawAugObs3D(:,:,:,obsInd), t-2*pi, [1 2], 3);
  else
    rawObsDatai = rotateData(g, rawAugObs3D(:,:,:,obsInd), t+2*pi, [1 2], 3);
  end
  rawObsDatai = shiftData(g, rawObsDatai, p, [1 2]);
  
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsDatai);
  
  % Exclude target set
  obj.obsForIntr(:,:,:,i) = max(obj.obsForIntr(:,:,:,i), -obj.target);
end
end