function [hc, ho, hn] = ...
  plotVehicles(Q, tInds, g, hc, ho, hn, colors, capture_radius)
% plotVehicles(Q, hc, ho, colors, capture_radius)
%     Updates the plot in the SPP simulation

for veh = 1:length(Q)
  if ~isempty(tInds{veh})
    % Plot capture radius
    if isempty(hc{veh})
      hc{veh} = plotDisk( ...
        Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
    else
      [~, hc{veh}.XData, hc{veh}.YData] = plotDisk( ...
        Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
    end
    
    % Plot induced obstacles
    [g2D, data2D] = ...
      proj(g, Q{veh}.data.cylObs3D(:,:,:,tInds{veh}), [0 0 1]);
    if isempty(ho{veh})
      ho{veh} = visSetIm(g2D, data2D, colors(veh, :));
      ho{veh}.LineStyle = '--';
    else
      ho{veh}.ZData = data2D;
    end
    
    % Plot nominal trajectory
    if isempty(hn{veh})
      hn{veh} = plot(Q{veh}.data.nomTraj(1,tInds{veh}), ...
        Q{veh}.data.nomTraj(2,tInds{veh}), '*', 'color', colors(veh,:));
    else
      hn{veh}.XData = Q{veh}.data.nomTraj(1,tInds{veh});
      hn{veh}.YData = Q{veh}.data.nomTraj(2,tInds{veh});
    end
    
    % Plot position
    Q{veh}.plotPosition(colors(veh, :));
  end
end
end