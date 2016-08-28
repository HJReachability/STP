function computeObsForRTT(obj, SPPP, RTTRS)
% vehicle = computeCylObs(vehicle, schemeData, rawCylObs, debug)
%     Computes cylindrical obstacles assuming the RTT method for SPP with
%     disturbances or SPP with intruders
% 
%     Takes 2D RTTRS augmented with capture radius as input

g2D = SPPP.g2D;
g = SPPP.g;

% Migrate RTTRS set
[RTTRS2D_g, RTTRS2D] = proj(RTTRS.g, -RTTRS.data, [0 0 1]);
RTTRS2D = migrateGrid(RTTRS2D_g, RTTRS2D, g2D);
RTTRS2D = addCRadius(g2D, RTTRS2D, SPPP.Rc + RTTRS.trackingRadius);

obj.obsForRTT_tau = obj.nomTraj_tau;
obj.obsForRTT = zeros([g.N' length(obj.nomTraj_tau)]);

for i = 1:length(obj.nomTraj_tau)
  % Rotate and shift raw obstacles
  p = obj.nomTraj(1:2,i);
  t = obj.nomTraj(3,i);
  rawObsDatai = rotateData(g2D, RTTRS2D, t, [1 2], []);
  rawObsDatai = shiftData(g2D, rawObsDatai, p, [1 2]);
  
  obj.obsForRTT(:,:,:,i)= repmat(rawObsDatai, [1 1 g.N(3)]);
  
  % Exclude target set
  obj.obsForRTT(:,:,:,i) = max(obj.obsForRTT(:,:,:,i), -obj.target);
end

end