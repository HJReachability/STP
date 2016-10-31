function obstacles = updateObstacles(obstacles, newObs)
% obstacles = gatherObstacles(vehicles, schemeData, tau, obs_type)
%     Gathers obstacles by combining obstacles in the field obs_type of each
%     vehicle in the vehicles list

% Determine time bound
min_tau = max( min(newObs.tau), min(obstacles.tau) ) - small;
max_tau = min( max(newObs.tau), max(obstacles.tau) ) + small;

% Determine indices within the time bound
global_tau_inds = obstacles.tau > min_tau & obstacles.tau < max_tau;
obs_tau_inds = newObs.tau > min_tau & newObs.tau < max_tau;

% Take union with previous obstacles for obstacles within the time bound
obstacles.data(:,:,:,global_tau_inds) = ...
  min(obstacles.data(:,:,:,global_tau_inds), newObs.data(:,:,:,obs_tau_inds));
end