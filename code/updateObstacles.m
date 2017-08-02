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
  
  %combine old and new obstacles in the overlapping time indices
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
  newObsSmaller = min(newObs(:,:,:,smaller_inds), staticObsSmaller); %all obstacles in the smaller time indices
 
  %combine obstacles by concatenating newObsSmaller with current obstacles
  obstacles.data = cat(4, newObsSmaller, obstacles.data);
end

% No need to update obstacle data for larger time indices    
end