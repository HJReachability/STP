function addObs2D(obj, SPPP, RTTRS, CARS, rawAugObs)
% addObs2D(obj, SPPP, RTTRS, CARS, rawAugmentedObs)
%     Adds 2D obstacles for visualization. If replanning was done, then the
%     appropriately augmented obstacles are added for the times before
%     replanning

g = SPPP.g;
g2D = SPPP.g2D;

% If there's a replan time, then migrate flattened augmented obstacles
replan = false;
if ~isempty(SPPP.tReplan) && nargin > 4
  replan = true;
  rawAugObs2D = migrateGrid(rawAugObs.g2D, rawAugObs.data2D, g2D);
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
  % Shift and rotation amounts
  p = obj.nomTraj(1:2, trajInd);
  t = obj.nomTraj(3, trajInd);
  
  if replan && obj.nomTraj_tau(i) < SPPP.tReplan
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
    
    rawObsDatai = rotateData(g2D, rawAugObs2D(:,:,obsInd), t, [1 2], []);
    rawObsDatai = shiftData(g2D, rawObsDatai, p, [1 2]);
    
  else
    rawObsDatai = rotateData(g2D, rawObs2D, t, [1 2], []);
    rawObsDatai = shiftData(g2D, rawObsDatai, p, [1 2]);
  end
  
  % Subtract target set
  obj.obs2D(:,:,i) = max(rawObsDatai, -obj.target);
end
end