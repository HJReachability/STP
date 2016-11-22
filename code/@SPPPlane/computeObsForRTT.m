function computeObsForRTT(obj, SPPP, RTTRS, nomTraj, nomTraj_tau)
% vehicle = computeCylObs(vehicle, schemeData, rawCylObs, debug)
%     Computes cylindrical obstacles assuming the RTT method for SPP with
%     disturbances or SPP with intruders
% 
%     Takes 2D RTTRS augmented with capture radius as input

if nargin < 4
  nomTraj = obj.nomTraj;
  nomTraj_tau = obj.nomTraj_tau;
end

% Project RTTRS into 2D
[RTTRS_g2D, RTTRS2D] = proj(RTTRS.g, -RTTRS.data, [0 0 1]);
R_augment = 1.1*(SPPP.Rc + RTTRS.trackingRadius); % Amount to augment RTTRS by

% Create slightly bigger grid to augment the RTTRS
small_g2D_min = RTTRS_g2D.min - R_augment;
small_g2D_max = RTTRS_g2D.max + R_augment;
small_g2D_N = ceil((small_g2D_max - small_g2D_min) ./ ...
  (RTTRS_g2D.max - RTTRS_g2D.min) .* RTTRS_g2D.N);
small_g2D = createGrid(small_g2D_min, small_g2D_max, small_g2D_N);

% Migrate RTTRS set
RTTRS2D = migrateGrid(RTTRS_g2D, RTTRS2D, small_g2D);
RTTRS2D = addCRadius(small_g2D, RTTRS2D, R_augment);
RTTRS2D = migrateGrid(small_g2D, RTTRS2D, SPPP.g2D);

obj.obsForRTT_tau = nomTraj_tau;
obj.obsForRTT = zeros([SPPP.g.N' length(nomTraj_tau)]);

for i = 1:length(nomTraj_tau)
  % Rotate and shift raw obstacles
  p = nomTraj(1:2,i);
  t = nomTraj(3,i);
  rawObsDatai = rotateData(SPPP.g2D, RTTRS2D, t, [1 2], []);
  rawObsDatai = shiftData(SPPP.g2D, rawObsDatai, p, [1 2]);
  
  obj.obsForRTT(:,:,:,i)= repmat(rawObsDatai, [1 1 SPPP.g.N(3)]);
  
  % Exclude target set
  obj.obsForRTT(:,:,:,i) = max(obj.obsForRTT(:,:,:,i), -obj.target);
end

end