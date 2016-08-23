function vehicle = computeCylObs(vehicle, schemeData, augRTTRS2D)
% vehicle = computeCylObs(vehicle, schemeData, rawCylObs, debug)
%     Computes cylindrical obstacles assuming the RTT method for SPP with
%     disturbances or SPP with intruders
% 
%     Takes 2D RTTRS augmented with capture radius as input

nomTraj_tau = vehicle.data.nomTraj_tau;
vehicle.data.cylObs_tau = vehicle.data.nomTraj_tau;
vehicle.data.cylObs = zeros([schemeData.grid.N' length(nomTraj_tau)]);

for i = 1:length(vehicle.data.nomTraj_tau)
  % Rotate and shift raw obstacles
  p = vehicle.data.nomTraj(1:2,i);
  t = vehicle.data.nomTraj(3,i);
  rawObsDatai = rotateData(augRTTRS2D.g, augRTTRS2D.data, t, [1 2], []);
  rawObsDatai = shiftData(augRTTRS2D.g, rawObsDatai, p, [1 2]);
  
  vehicle.data.cylObs(:,:,:,i)= repmat(rawObsDatai, [1 1 schemeData.grid.N(3)]);
  
  % Exclude target set
  vehicle.data.cylObs(:,:,:,i) = ...
    max(vehicle.data.cylObs(:,:,:,i), -vehicle.data.target);
end

end