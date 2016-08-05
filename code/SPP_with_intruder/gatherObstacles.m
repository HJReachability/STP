function obstacles = gatherObstacles(vehicles, schemeData, tau)
% obstacles = gatherObstacles(higherPVehicles, schemeData, tau);
%     Computes the union of induced obstacles by a set of vehicles, with
%     time-stamps synced to tau. This set of obstacles is used to compute BRS1

% If there are no vehicles in the input, and there is no static obstacles...
if isempty(vehicles)
  obstacles = inf(schemeData.grid.N');
  return
end

% Convert tau to absolute time
tau = flip(-tau);

% Initialize empty obstacle
obstacles = inf([schemeData.grid.N', length(tau)]);

% Go through each vehicle in the input
for i = 1:length(vehicles)
  [~, tau_inds, obs_inds] = intersect(tau, vehicles{i}.data.augObsFRS_tau);
  obstacles(:,:,:,tau_inds) = min(obstacles(:,:,:,tau_inds), ...
    vehicles{i}.data.augFlatObsBRS(:,:,:,obs_inds));
end
end