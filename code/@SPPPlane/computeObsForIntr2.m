function computeObsForIntr2(obj, g, rawAugObs)
% computeObsForIntr2(obj, g, CARS, rawAugObs)
%     Computes the FRS + BRS + IES obstacles assuming the RTT method
%     for SPP with intruders method 2

% Migrate the raw augmented obstacles
rawObsFRS = migrateGrid(rawAugObs.g, rawAugObs.FRS3D, g);
rawObsBRS = migrateGrid(rawAugObs.g, rawAugObs.BRS3D, g);
rawObsIES = migrateGrid(rawAugObs.g, rawAugObs.IES, g);

obj.obsForIntr_tau = obj.nomTraj_tau;
obj.obsForIntr = inf([g.N' length(obj.nomTraj_tau)]);
len_tIAT = size(rawObsFRS);
len_tIAT = len_tIAT(end);

for i = 1:length(obj.nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(obj.nomTraj_tau))
  
  % Determine trajectory and obstacle indices for each of the raw obstacles
  trajIndIES = i;
  trajIndFRS = max(1, i-len_tIAT+1);
  trajIndBRS = min(length(obj.nomTraj_tau), i+len_tIAT-1);
  
  % IES always has the same obstacle index
  obsIndFRS = min(i, len_tIAT);
  obsIndBRS = min(length(obj.nomTraj_tau)-i+1, len_tIAT);
  
  % Add IES obstacle
  pIES = obj.nomTraj(1:2, trajIndIES);
  tIES = obj.nomTraj(3, trajIndIES);
  rawObsIESi = rotateData(g, rawObsIES, tIES, [1 2], 3);
  rawObsIESi = shiftData(g, rawObsIESi, pIES, [1 2]);
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsIESi);
  
  % Add FRS obstacle
  pFRS = obj.nomTraj(1:2, trajIndFRS);
  tFRS = obj.nomTraj(3, trajIndFRS);
  rawObsFRSi = rotateData(g, rawObsFRS(:,:,:,obsIndFRS), tFRS, [1 2], 3);
  rawObsFRSi = shiftData(g, rawObsFRSi, pFRS, [1 2]);
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsFRSi);  
  
  % Add BRS obstacle
  pBRS = obj.nomTraj(1:2, trajIndBRS);
  tBRS = obj.nomTraj(3, trajIndBRS);
  rawObsBRSi = rotateData(g, rawObsBRS(:,:,:,obsIndBRS), tBRS, [1 2], 3);
  rawObsBRSi = shiftData(g, rawObsBRSi, pBRS, [1 2]);
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsBRSi);    
  
  %% Exclude target set
  obj.obsForIntr(:,:,:,i) = max(obj.obsForIntr(:,:,:,i), -obj.target);
end
end