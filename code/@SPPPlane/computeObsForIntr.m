function computeObsForIntr(obj, g, CARS, rawAugObs)
% computeAugObs(obj, g, CARS, rawAugObs)
%     Computes the FRS + CR + BRS-augmented obstacles assuming the RTT method
%     for SPP with intruders

% Migrate the raw augmented obstacles
rawAugObs3D = cell(length(CARS.tau), 1);
for i = 1:length(CARS.tau)
  rawAugObs3D{i} =  migrateGrid(rawAugObs.g, rawAugObs.datas{i}, g);
end

obj.obsForIntr_tau = obj.nomTraj_tau;
obj.obsForIntr = inf([g.N' length(obj.nomTraj_tau)]);

for i = 1:length(obj.nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(obj.nomTraj_tau))
  
  for j = 1:length(CARS.tau)
    trajInd = i-j+1;
    obsFRSInd = length(CARS.tau);
    
    % If trajInd is before start of trajectory, then shift the FRS index instead
    if trajInd < 1
      obsFRSInd = length(CARS.tau) + trajInd - 1;
      trajInd = 1;
    end
    
    % Rotate and shift data to nominal trajectory
    p = obj.nomTraj(1:2,trajInd);
    t = obj.nomTraj(3,trajInd);
    rawObsDatai = rotateData(g, rawAugObs3D{obsFRSInd}(:,:,:,j), t, [1 2], 3);
    rawObsDatai = shiftData(g, rawObsDatai, p, [1 2]);
    
    % Take union of the tIAT reachable sets
    obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), rawObsDatai);
  end
  
  %% Exclude target set
  obj.obsForIntr(:,:,:,i) = max(obj.obsForIntr(:,:,:,i), -obj.target);
end
end