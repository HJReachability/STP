function plotTargetSets(Q, g2D, colors)

for veh = 1:length(Q)
  target2D = shapeSphere(g2D, Q{veh}.targetCenter, Q{veh}.targetR);
  ht = visSetIm(g2D, target2D, colors(veh,:));
  ht.LineWidth = 3;
  hold on
end

end