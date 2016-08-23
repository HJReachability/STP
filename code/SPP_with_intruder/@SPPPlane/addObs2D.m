function addObs2D(obj, SPPP, RTTRS, CARS, rawAugmentedObs)

g = SPPP.g;
g2D = SPPP.g2D;

% If there's a replan time, then migrate and flatten augmented obstacles
if ~isempty(SPPP.tReplan)
  tauIAT = rawAugmentedObs.tauIAT;
  rawAugmentedObs2D = zeros([g2D.N' length(tauIAT)]);
  for i = 1:length(tauIAT);
    [~, rawAugmentedObs2D(:,:,i)] = ...
      proj(g, rawAugmentedObs.data(:,:,:,i), [0 0 1]);
  end
end

% Migrate and add capture radius to RTTRS
rawObs = migrateGrid(RTTRS.g, RTTRS.data, g);
[~, rawObs2D] = proj(g, rawObs, [0 0 1]);
rawObs2D = addCRadius(g2D, rawObs2D, CARS.Rc);

% Initialize 2D obstacles
obj.obs2D_tau = obj.nomTraj_tau;
obj.obs2D = zeros([g2D.N' length(obj.nomTraj_tau)]);

small = 1e-4;
trajInd = 0;
for i = 1:length(obj.nomTraj_tau)
  
  p = obj.nomTraj(1:2, trajInd);
  
  if obj.nomTraj_tau(i) < tReplan
    % If we haven't replanned yet, add the augmented obstacles (only applicable
    % to SPP with intruder
    
    % For the first tauIAT time steps, use the i-step FRS projection
    if obj.nomTraj_tau(i)-min(obj.nomTraj_tau) < max(tauIAT)
      obsInd = find(tauIAT > obj.nomTraj_tau(i)-min(obj.nomTraj_tau)-small & ...
        tauIAT < obj.nomTraj_tau(i)-min(obj.nomTraj_tau)+small);
    else
      obsInd = length(tauIAT);
      trajInd = trajInd + 1;
    end
    trajInd = max(trajInd, 1);
    
    rawObsDatai = shiftData(g2D, rawAugmentedObs2D(:,:,obsInd), p, [1 2]);
    
  else
    
    rawObsDatai = shiftData(g2D, rawObs2D, p, [1 2]);
  end
  
  % Subtract target set
  obj.obs2D(:,:,i) = max(rawObsDatai, -obj.target);
end
end