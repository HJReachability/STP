function saveReplanData(Q, Qintr, replan_filename)
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
  Qnew{veh}.data.targetRsmall = Q{veh}.data.targetRsmall;
  Qnew{veh}.data.targetR = Q{veh}.data.targetR;  
  Qnew{veh}.data.target = Q{veh}.data.target;
  Qnew{veh}.data.targetsm = Q{veh}.data.targetsm;

  Qnew{veh}.data.vReserved = Q{veh}.data.vReserved;
  Qnew{veh}.data.wReserved = Q{veh}.data.wReserved;
  
  Qnew{veh}.data.tauBR = Q{veh}.data.tauBR;
  Qnew{veh}.data.replan = Q{veh}.data.replan;
  
  % For vehicles that don't need to replan, copy over nominal trajectory
  if ~Q{veh}.data.replan
    Qnew{veh}.data.nomTraj = Q{veh}.data.nomTraj;
    Qnew{veh}.data.nomTraj_tau = Q{veh}.data.nomTraj_tau;
  end
end

[Q1, Q2, Q3, Q4] = Qnew{:};

save(replan_filename, 'Q1', 'Q2', 'Q3', 'Q4', 'Qintr', '-v7.3')
end