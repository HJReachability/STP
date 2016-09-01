function plotTargetSets(Q, colors)
% plotTargetSets(Q, colors)
%   Plots the target sets of the vehicles in Q

if ~iscell(colors)
  temp = colors;
  colors = cell(1, length(Q));
  for i = 1:length(Q)
    colors{i} = temp(i,:);
  end
end
for veh = 1:length(Q)
  plotDisk(Q{veh}.targetCenter, Q{veh}.targetR, ...
    'linewidth', 3, 'color', colors{veh});
  hold on
end

end