function obstacles = gatherObstacles(g, obsSet, obsSet_tau, common_tau)
% Convert tau to absolute time
common_tau = flip(-common_tau);

% Initialize empty obstacle
obstacles = inf([g, length(common_tau)]);

small = 1e-4;
% Go through each vehicle in the input
for i = 1:length(obsSet)
  % Determine time bound
  min_tau = min(obsSet_tau{i}) - small;
  max_tau = min(small, max(obsSet_tau{i}) + small);
  
  % Determine indices within the time bound
  tau_inds = common_tau > min_tau & common_tau < max_tau;
  obs_inds = obsSet_tau{i} > min_tau & obsSet_tau{i} < max_tau;

  % Take union with previous obstacles for obstacles within the time bound
  obstacles(:,:,:,tau_inds) = ...
    min(obstacles(:,:,:,tau_inds), obsSet{i}(:,:,:,obs_inds));
end

% Flip the obstacles so that it goes backwards in time; this is needed since the
% BRS computation goes backwards in time
obstacles = flip(obstacles, 4);
end