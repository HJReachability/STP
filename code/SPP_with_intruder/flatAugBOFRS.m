function vehicle = flatAugBOFRS(vehicle, schemeData, capture_radius)
% vehicle = flatAugBOFRS(vehicle, capture_radius)
%     Flattens and augments the tIAT-FRS-augmented obstacles, adds the capture
%     radius, and then extends the 2D shape into 3D. Populates the .flatObs2D
%     and .cylObs3D fields of vehicle.data
%
%     Thin wrapper around flatAugObs()

vehicle.data.cylObs3D_tau = vehicle.data.augObsFRS_tau;

[~, vehicle.data.flatObs2D, vehicle.data.cylObs3D] = flatAugObs( ...
  schemeData.grid, vehicle.data.augObsFRS, vehicle.data.augObsFRS_tau, ...
  capture_radius);
end