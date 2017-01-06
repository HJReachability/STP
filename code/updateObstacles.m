function obstacles = updateObstacles(obstacles, newObs_tau, newObs, staticObs)
% obstacles = gatherObstacles(vehicles, schemeData, tau, obs_type)
%     Gathers obstacles by combining obstacles in the field obs_type of each
%     vehicle in the vehicles list

if nargin < 4
  staticObs = [];
end

if numDims(staticObs) == 2
  staticObs = repmat(staticObs, [1 1 size(newObs, 3)]);
end

small = 1e-4;

% Update obstacle data for overlapping time indices
overlap_inds_newObs = newObs_tau > min(obstacles.tau)-small & ...
  newObs_tau < max(obstacles.tau)+small;
if any(overlap_inds_newObs)
  overlap_inds_oldObs = obstacles.tau > min(newObs_tau)-small & ...
    obstacles.tau < max(newObs_tau)+small;  
  obstacles.data(:,:,:,overlap_inds_oldObs) = ...
    min(obstacles.data(:,:,:,overlap_inds_oldObs), ...
    newObs(:,:,:,overlap_inds_newObs));
else
  error('There must be overlap between the time stamps!')
end

% Update obstacle data for smaller time indices
smaller_inds = newObs_tau < min(obstacles.tau)-small;
if any(smaller_inds)
  obstacles.tau = [newObs_tau(smaller_inds) obstacles.tau];

  staticObsSmaller = repmat(staticObs, [1 1 1 nnz(smaller_inds)]);
  newObsSmaller = min(newObs(:,:,:,smaller_inds), staticObsSmaller);

  obstacles.data = cat(4, newObsSmaller, obstacles.data);
end

% Update obstacle data for larger time indices
larger_inds = newObs_tau > max(obstacles.tau)+small;
if any(larger_inds)
  obstacles.tau = [obstacles.tau newObs_tau(larger_inds)];
  
  staticObsLarger = repmat(staticObs, [1 1 1 nnz(larger_inds)]);
  newObsLarger = min(newObs(:,:,:,larger_inds), staticObsLarger);
  
  obstacles.data = cat(4, obstacles.data, newObsLarger);
end

% % Determine time bound
% min_tau = max( min(newObs_tau), min(tau) ) - small;
% max_tau = min( max(newObs_tau), max(tau) ) + small;
% 
% % Determine indices within the time bound
% global_tau_inds = tau > min_tau & tau < max_tau;
% obs_tau_inds = newObs_tau > min_tau & newObs_tau < max_tau;
% 
% % Take union with previous obstacles for obstacles within the time bound
% obstacles(:,:,:,global_tau_inds) = ...
%   min(obstacles(:,:,:,global_tau_inds), newObs(:,:,:,obs_tau_inds));
end