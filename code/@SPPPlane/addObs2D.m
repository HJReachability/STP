function addObs2D(obj, SPPP, RTTRS, CARS, rawAugObs)
% addObs2D(obj, SPPP, RTTRS, CARS, rawAugmentedObs)
%     Adds 2D obstacles for visualization. If replanning was done, then the
%     appropriately augmented obstacles are added for the times before
%     replanning

small = 1e-3;
nomTraj = obj.nomTraj;
nomTraj_tau = obj.nomTraj_tau;

% Use newer nominal trajectory if there's replanning
if ~isempty(obj.nomTraj_AR)
  % Determine the time steps at which there is new nominal trajectory
  ARinds = nomTraj_tau >= min(obj.nomTraj_AR_tau) - small;
  
  % Replace nominal trajectory
  nomTraj(ARinds) = [];
  nomTraj_tau(ARinds) = [];
  nomTraj = [nomTraj obj.nomTraj_AR];
  nomTraj_tau = [nomTraj_tau obj.nomTraj_AR_tau];
end

g = SPPP.g;
g2D = SPPP.g2D;

% If there's a replan time, then migrate flattened augmented obstacles
replan = false;
if ~isempty(SPPP.tReplan) && nargin > 3
  replan = true;
  rawAugObs2D = migrateGrid(rawAugObs.g2D, rawAugObs.FRS2D, g2D);
end

% Migrate and add capture radius to RTTRS
rawObs = migrateGrid(RTTRS.g, RTTRS.data, g);
[~, rawObs2D] = proj(g, rawObs, [0 0 1]);
rawObs2D = addCRadius(g2D, rawObs2D, SPPP.Rc);

% Initialize 2D obstacles
obj.obs2D_tau = nomTraj_tau;
obj.obs2D = zeros([g2D.N' length(nomTraj_tau)]);

small = 1e-4;

for i = 1:length(nomTraj_tau)
  % Shift and rotation amounts
  p = nomTraj(1:2, i);
  t = nomTraj(3, i);
  
  if replan && nomTraj_tau(i) < SPPP.tReplan
    % Before replanning:
    %     for the first tauIAT time steps, use the i-step FRS projection
    tauElapsed = nomTraj_tau(i) - min(nomTraj_tau);
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
  obj.obs2D(:,:,i) = shiftData(g2D, rawObsDatai, p, [1 2]);
end
end