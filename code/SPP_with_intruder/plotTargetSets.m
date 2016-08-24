function plotTargetSets(Q, g2D, colors)

for veh = 1:length(Q)
  plotDisk(Q{veh}.targetCenter, Q{veh}.targetR, 'linewidth', 3);
  hold on
end

end