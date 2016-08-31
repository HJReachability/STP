function addObs2D(obj, SPPP, RTTRS, CARS, rawAugObs)
% addObs2D(obj, SPPP, RTTRS, CARS, rawAugmentedObs)
%     Adds 2D obstacles for visualization. If replanning was done, then the
%     appropriately augmented obstacles are added for the times before
%     replanning

g = SPPP.g;
g2D = SPPP.g2D;

% If there's a replan time, then migrate flattened augmented obstacles
replan = false;
if ~isempty(SPPP.tReplan) && nargin > 3
  replan = true;
  rawAugObs2D = migrateGrid(rawAugObs.g2D, rawAugObs.data2D, g2D);
end

% Migrate and add capture radius to RTTRS
rawObs = migrateGrid(RTTRS.g, RTTRS.data, g);
[~, rawObs2D] = proj(g, rawObs, [0 0 1]);
rawObs2D = addCRadius(g2D, rawObs2D, SPPP.Rc);

% Initialize 2D obstacles
obj.obs2D_tau = obj.nomTraj_tau;
obj.obs2D = zeros([g2D.N' length(obj.nomTraj_tau)]);

small = 1e-4;

for i = 1:length(obj.nomTraj_tau)
  % Shift and rotation amounts
  p = obj.nomTraj(1:2, i);
  t = obj.nomTraj(3, i);
  
  if replan && obj.nomTraj_tau(i) < SPPP.tReplan
    % Before replanning:
    %     for the first tauIAT time steps, use the i-step FRS projection
    tauElapsed = obj.nomTraj_tau(i) - min(obj.nomTraj_tau);
    if tauElapsed < max(CARS.tau)
      obsInd = find(CARS.tau > tauElapsed-small & CARS.tau < tauElapsed+small);
    else
      obsInd = length(CARS.tau);
    end
    
    rawObsToRotate = rawAugObs2D(:,:,obsInd);
  else
    % After replanning: always use the same 2D obstacle
    rawObsToRotate = rawObs2D;
  end
  
  % Rotate and shift 2D obstacle
  rawObsDatai = rotateData(g2D, rawObsToRotate, t, [1 2], []);
  rawObsDatai = shiftData(g2D, rawObsDatai, p, [1 2]);
  
  % Subtract target set
  target2D = shapeSphere(g2D, obj.targetCenter, obj.targetR);
  obj.obs2D(:,:,i) = max(rawObsDatai, -target2D);
end
end