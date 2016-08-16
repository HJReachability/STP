function saveReplanData(Q, schemeData, tNow, safety_vals, safety_threshold)
% saveReplanData(Q, schemeData)
%     Removes most of the fields of vehicle objects, except for those needed to
%     do replanning after an intruder has comels

Qnew = cell(length(Q),1);
replan = false;
for i = 1:length(Q)
  % Basic class properties
  Qnew{i} = Plane(Q{i}.x, Q{i}.wMax, Q{i}.vrange, Q{i}.dMax);
  Qnew{i}.xhist = Q{i}.xhist;
  Qnew{i}.u = Q{i}.u;
  Qnew{i}.uhist = Q{i}.uhist;
  Qnew{i}.hpxpy = Q{i}.hpxpy;
  Qnew{i}.hpxpyhist = Q{i}.hpxpyhist;
  
  % Data
  Qnew{i}.data.targetCenter = Q{i}.data.targetCenter;
  Qnew{i}.data.target = Q{i}.data.target;
  Qnew{i}.data.targetsm = Q{i}.data.targetsm;
  Qnew{i}.data.vReserved = Q{i}.data.vReserved;
  Qnew{i}.data.wReserved = Q{i}.data.wReserved;
  
  % Determine which vehicles have been affected by intruder
  if ~replan && any(safety_vals(i, :) < safety_threshold)
    replan = true;
  end
  Qnew{i}.data.replan = replan;
  
  if ~replan
    fprintf('Re-populating augmented obstacles for vehicle %d\n', i)
    
    if ~exist('rawObsBRS', 'var')
      % Load obstacles
      fprintf('Loading ''raw'' obstacles...\n')
      load(Obs_filename)
      rawObsBRS.data = zeros([schemeData.grid.N' length(tauIAT)]);
      for i = 1:length(tauIAT)
        rawObsBRS.data(:,:,:,i) = ...
          migrateGrid(rawObs.g, rawObs.cylObsBRS(:,:,:,i), schemeData.grid);
      end
      rawObsBRS.tauIAT = tauIAT;
    end
    Qnew{i} = augmentObstacles(Qnew{i}, schemeData, rawObsBRS);
  end
end

[Q1, Q2, Q3, Q4] = Qnew{:};

save(sprintf('Replan_RS_%f.mat', now), 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', ...
  'tNow', '-v7.3')
end