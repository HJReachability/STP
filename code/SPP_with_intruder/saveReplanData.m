function saveReplanData(Q, schemeData, rawObs, tauIAT, tNow)
% saveReplanData(Q, schemeData)
%     Removes most of the fields of vehicle objects, except for those needed to
%     do replanning after an intruder has comels

Qnew = cell(length(Q),1);
for veh = 1:length(Q)
  % Basic class properties
  Qnew{veh} = Plane(Q{veh}.x, Q{veh}.wMax, Q{veh}.vrange, Q{veh}.dMax);
  Qnew{veh}.xhist = Q{veh}.xhist;
  Qnew{veh}.u = Q{veh}.u;
  Qnew{veh}.uhist = Q{veh}.uhist;
  Qnew{veh}.hpxpy = Q{veh}.hpxpy;
  Qnew{veh}.hpxpyhist = Q{veh}.hpxpyhist;
  
  % Data
  Qnew{veh}.data.targetCenter = Q{veh}.data.targetCenter;
  Qnew{veh}.data.target = Q{veh}.data.target;
  Qnew{veh}.data.targetsm = Q{veh}.data.targetsm;
  Qnew{veh}.data.vReserved = Q{veh}.data.vReserved;
  Qnew{veh}.data.wReserved = Q{veh}.data.wReserved;
  Qnew{veh}.data.replan = Q{veh}.data.replan;
  
  if ~Q{veh}.data.replan
    Qnew{veh}.data.nomTraj = Q{veh}.data.nomTraj;
    Qnew{veh}.data.nomTraj_tau = Q{veh}.data.nomTraj_tau;
    
    fprintf('Re-populating augmented obstacles for vehicle %d\n', veh)
    
    if ~exist('rawObsBRS', 'var')
      % Load obstacles
      fprintf('Populating ''raw'' obstacles...\n')
      rawObsBRS.data = zeros([schemeData.grid.N' length(tauIAT)]);
      for i = 1:length(tauIAT)
        rawObsBRS.data(:,:,:,i) = ...
          migrateGrid(rawObs.g, rawObs.cylObsBRS(:,:,:,i), schemeData.grid);
      end
      rawObsBRS.tauIAT = tauIAT;
    end
    Qnew{veh} = augmentObstacles(Qnew{veh}, schemeData, rawObsBRS);
  end
end

[Q1, Q2, Q3, Q4] = Qnew{:};

save(sprintf('Replan_RS_%f.mat', now), 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', ...
  'tNow', '-v7.3')
end