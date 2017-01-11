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

R_augment = 1.1*(SPPP.Rc + RTTRS.trackingRadius); % Amount to augment RTTRS by
[g2D, RTTRS2D] = migrateRTTRS(RTTRS, R_augment);

% Initialize 2D obstacles
obj.obs2D_tau = nomTraj_tau;
obj.obs2D = zeros([SPPP.g2D.N' length(nomTraj_tau)]);

% Initialize 3D obstacles
obj.obsForRTT_tau = nomTraj_tau;
obj.obsForRTT = zeros([SPPP.g.N' length(nomTraj_tau)]);

for i = 1:length(nomTraj_tau)
  % Rotate and shift raw obstacles
  p = nomTraj(1:2,i);
  t = nomTraj(3,i);
  obsi_rot = rotateData(g2D, RTTRS2D, t, [1 2], []);
  obsi_gShift = shiftGrid(g2D, p);
  obsi = migrateGrid(obsi_gShift, obsi_rot, SPPP.g2D);
  
  obj.obs2D(:,:,i) = obsi;  
  obj.obsForRTT(:,:,:,i) = repmat(obsi, [1 1 SPPP.g.N(3)]);
  
  % Exclude target set
  obj.obsForRTT(:,:,:,i) = max(obj.obsForRTT(:,:,:,i), -obj.target);
end

end