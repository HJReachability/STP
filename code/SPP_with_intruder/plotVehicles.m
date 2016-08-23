function [hc, ho, hn] = ...
  plotVehicles(Q, tInds, g2D, hc, ho, hn, colors, capture_radius)
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
    if isempty(ho{veh})
      ho{veh} = visSetIm(g2D, Q{veh}.obs2D(:,:,tInds{veh}), colors(veh, :));
      ho{veh}.LineStyle = '--';
    else
      ho{veh}.ZData = Q{veh}.obs2D(:,:,tInds{veh});
    end
    
    % Plot nominal trajectory
    if isempty(hn{veh})
      hn{veh} = plot(Q{veh}.nomTraj(1,tInds{veh}), ...
        Q{veh}.nomTraj(2,tInds{veh}), '*', 'color', colors(veh,:));
    else
      hn{veh}.XData = Q{veh}.nomTraj(1,tInds{veh});
      hn{veh}.YData = Q{veh}.nomTraj(2,tInds{veh});
    end
    
    % Plot position
    Q{veh}.plotPosition(colors(veh, :));
  end
end
end