function saveReplanData(Q, schemeData, tNow)
% saveReplanData(Q, schemeData)
%     Removes most of the fields of vehicle objects, except for those needed to
%     do replanning after an intruder has come

Qnew = cell(length(Q),1);
for i = 1:length(Q)
  % Basic class properties
  Qnew{i} = Plane(Q{i}.x, Q{i}.wMax, Q{i}.vrange, Q{i}.dMax);
  Qnew{i}.xhist = Q{i}.xhist;
  Qnew{i}.u = Q{i}.u;
  Qnew{i}.uhist = Q{i}.uhist;
  Qnew{i}.hpxpy = Q{i}.hpxpy;
  Qnew{i}.hpxpyhist = Q{i}.hpxpyhist;
  
  % Data
  Qnew{i}.data.target = Q{i}.data.target;
  Qnew{i}.data.targetsm = Q{i}.data.targetsm;
  Qnew{i}.data.vReserved = Q{i}.data.vReserved;
  Qnew{i}.data.wReserved = Q{i}.data.wReserved;
  Qnew{i}.data.RTT_radius = Q{i}.data.RTT_radius;
end

[Q1, Q2, Q3, Q4] = Qnew{:};

save('SPPwIntruder_RS_bare.mat', 'Q1', 'Q2', 'Q3', 'Q4', 'schemeData', ...
  'tNow', '-v7.3')
end