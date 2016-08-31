function plotTargetSets(Q, colors)
% plotTargetSets(Q, colors)
%   Plots the target sets of the vehicles in Q

for veh = 1:length(Q)
  plotDisk(Q{veh}.targetCenter, Q{veh}.targetR, ...
    'linewidth', 3, 'color', colors(veh,:));
  hold on
end

end