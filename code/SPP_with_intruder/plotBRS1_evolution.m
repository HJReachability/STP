function plotBRS1_evolution(Q, veh, schemeData)

theta0 = Q{veh}.xhist(3,1);

figure
for i = length(Q{veh}.data.BRS1_tau):-1:1
  % Plot BRS
  [g2D, BRS2D] = ...
    proj(schemeData.grid, Q{veh}.data.BRS1(:,:,:,i), [0 0 1], theta0);
  visSetIm(g2D, BRS2D);
  hold on
  
  % Plot obstacle at the same time
  small = 1e-4;
  t = Q{veh}.data.BRS1_tau(i);
  for j = 1:veh-1
    oInd = find(Q{j}.data.augFlatObsBRS_tau < t+small & ...
      Q{j}.data.augFlatObsBRS_tau > t-small);
    
    if ~isempty(oInd)
      [~, Obs2D] = proj(schemeData.grid, ...
        Q{j}.data.augFlatObsBRS(:,:,:,oInd), [0 0 1], theta0);
      visSetIm(g2D, Obs2D, 'k');
    end
  end
  
  drawnow;
  hold off
end
end