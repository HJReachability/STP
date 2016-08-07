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

small = 1e-4;
% Go through each vehicle in the input
for i = 1:length(vehicles)
  % Determine time bound
  min_tau = min(vehicles{i}.data.augObsFRS_tau) - small;
  max_tau = min(small, max(vehicles{i}.data.augObsFRS_tau) + small);
  
  % Determine indices within the time bound
  tau_inds = tau > min_tau & tau < max_tau;
  obs_inds = vehicles{i}.data.augObsFRS_tau > min_tau & ...
    vehicles{i}.data.augObsFRS_tau < max_tau;

  % Take union with previous obstacles for obstacles within the time bound
  obstacles(:,:,:,tau_inds) = min(obstacles(:,:,:,tau_inds), ...
    vehicles{i}.data.augFlatObsBRS(:,:,:,obs_inds));
end

% Flip the obstacles so that it goes backwards in time; this is needed since the
% BRS computation goes backwards in time
obstacles = flip(obstacles, 4);
end