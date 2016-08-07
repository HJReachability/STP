function obstacles = gatherFlatObsBRS(vehicles, schemeData, tau)
% obstacles = gatherFlatObsBRS(vehicles, schemeData, tau)
%     Computes the union of induced obstacles by a set of vehicles, with
%     time-stamps synced to tau. This set of obstacles is used to compute BRS1
%
%     Thin wrapper around gatherObstacles

% If there are no vehicles in the input, and there is no static obstacles...
if isempty(vehicles)
  obstacles = inf(schemeData.grid.N');
  return
end

% Gather the obstacle set
obsSet = cell(length(vehicles), 1);
obsSet_tau = cell(length(vehicles), 1);

for i = 1:length(obsSet)
  obsSet{i} = vehicles{i}.data.augObsFRS;
  obsSet_tau{i} = vehicles{i}.data.augObsFRS_tau;
end

% Compute the obstacles
obstacles = gatherObstacles(schemeData.grid, obsSet, obsSet_tau, tau);
end