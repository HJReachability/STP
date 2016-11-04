function obstacles = updateObstacles(tau, obstacles, newObs_tau, newObs)
% obstacles = gatherObstacles(vehicles, schemeData, tau, obs_type)
%     Gathers obstacles by combining obstacles in the field obs_type of each
%     vehicle in the vehicles list

small = 1e-4;

% Determine time bound
min_tau = max( min(newObs_tau), min(tau) ) - small;
max_tau = min( max(newObs_tau), max(tau) ) + small;

% Determine indices within the time bound
global_tau_inds = tau > min_tau & tau < max_tau;
obs_tau_inds = newObs_tau > min_tau & newObs_tau < max_tau;

% Take union with previous obstacles for obstacles within the time bound
obstacles(:,:,:,global_tau_inds) = ...
  min(obstacles(:,:,:,global_tau_inds), newObs(:,:,:,obs_tau_inds));
end