function plotBRS1_evolution(Q, veh, schemeData, obs_field)
% plotBRS1_evolution(Q, veh, schemeData, obs_field)
%     Plots the backward time-evolution of BRS1 and and the induced obstacles
%     given in the field obs_field

theta0 = Q{veh}.xhist(3,1);

figure
for i = length(Q{veh}.data.BRS1_tau):-1:1
  % Plot BRS
  [g2D, BRS2D] = ...
    proj(schemeData.grid, Q{veh}.data.BRS1(:,:,:,i), [0 0 1], theta0);
  visSetIm(g2D, BRS2D);
  hold on
  
  % Plot obstacle at each time step
  small = 1e-4;
  t = Q{veh}.data.BRS1_tau(i);

  for j = 1:veh-1
    % Determine the corresponding time index in the obstacles
    oInd = find(Q{j}.data.(sprintf('%s_tau', obs_field)) < t+small & ...
      Q{j}.data.(sprintf('%s_tau', obs_field)) > t-small);
    
    % If a time index is found, plot the obstacle at that time step
    if ~isempty(oInd)
      [~, Obs2D] = proj(schemeData.grid, ...
        Q{j}.data.(obs_field)(:,:,:,oInd), [0 0 1], theta0);
      visSetIm(g2D, Obs2D, 'k');
    end
  end
  title(sprintf('t = %f', Q{veh}.data.BRS1_tau(i)))
  drawnow;
  hold off

end
end