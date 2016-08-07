function vehicle = flatAugBaseObs(vehicle, schemeData, capture_radius)
% vehicle = flatAugBOFRS(vehicle, capture_radius)
%     Flattens the base obstacles, adds the capture radius, and then extends the
%     2D shape into 3D. Populates the .flatBO2D and .cylBO3D fields of
%     vehicle.data
%
%     Thin wrapper around flatAugObs()

vehicle.data.cylBO3D_tau = vehicle.data.baseObs_tau;

[~, vehicle.data.flatBO2D, vehicle.data.cylBO3D] = flatAugObs( ...
  schemeData.grid, vehicle.data.baseObs, vehicle.data.baseObs_tau, ...
  capture_radius);
end