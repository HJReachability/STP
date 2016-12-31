numVeh = length(Q);
spC = ceil(sqrt(numVeh))+1;
spR = ceil(numVeh/spC);

figure
extraArgs.deleteLastPlot = false;
for veh = 1:numVeh
  subplot(spR, spC, veh)
  visSetIm(SPPP.g2D, Q{veh}.obs2D, 'r', 0, extraArgs);
end

for veh = 1:numVeh
  disp(veh)
  tic
  load(sprintf('SPPProblem_SF_intr_3/Plane_data/Plane%d.mat', veh))
  [~, Q{veh}.obs2D] = proj(SPPP.g, Qthis.obsForIntr, [0 0 1]);
  toc
end