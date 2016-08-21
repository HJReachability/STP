function plotTargetSets(Q, g, colors)

for veh = 1:length(Q)
  [g2D, data2D] = proj(g, Q{veh}.data.target, [0 0 1]);
  ht = visSetIm(g2D, data2D, colors(veh,:));
  ht.LineWidth = 3;
  hold on
end

end