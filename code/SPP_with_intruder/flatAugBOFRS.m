function vehicle = flatAugBOFRS(vehicle, schemeData, capture_radius)
% vehicle = flatAugBOFRS(vehicle, capture_radius)
%     Flattens and augments the tIAT-FRS-augmented obstacles, adds the capture
%     radius, and then extends the 2D shape into 3D. Populates the .flatObs2D
%     and .cylObs3D fields of vehicle.data
%

%% Parameters for adding capture radius
g2D = proj(schemeData.grid, [], [0 0 1]);

%% Initialize augmented-capture-radius flat obstacles
vehicle.data.flatObs2D = zeros([g2D.N' length(vehicle.data.augObsFRS_tau)]);
vehicle.data.cylObs3D = ...
  zeros([schemeData.grid.N' length(vehicle.data.augObsFRS_tau)]);

for i = 1:length(vehicle.data.augObsFRS_tau)
  [~, obs2D] = proj(schemeData.grid, vehicle.data.augObsFRS(:,:,:,i), [0 0 1]);
  vehicle.data.flatObs2D(:,:,i) = addCRadius(g2D, obs2D, capture_radius);
  vehicle.data.cylObs3D(:,:,:,i) = ...
    repmat(vehicle.data.flatObs2D(:,:,i), [1 1 schemeData.grid.N(3)]);
end
end