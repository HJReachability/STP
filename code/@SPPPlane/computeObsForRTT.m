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
RTTRS2D = migrateRTTRS(RTTRS, target_g2D, R_augment);

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