function [hc, ho, hn, ht] = ...
  plotVehicles(Q, tInds, g2D, hc, ho, hn, ht, colors, capture_radius)
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
    if ~isempty(Q{veh}.obs2D)
        if isempty(ho{veh})
          ho{veh} = visSetIm(g2D, Q{veh}.obs2D(:,:,tInds{veh}), colors(veh, :));
          ho{veh}.LineStyle = '--';
        else
          ho{veh}.ZData = Q{veh}.obs2D(:,:,tInds{veh});
        end
    end
    
    % Plot nominal trajectory
    x_nom = Q{veh}.nomTraj(1,tInds{veh});
    y_nom = Q{veh}.nomTraj(2,tInds{veh});
    if isempty(hn{veh})
      hn{veh} = plot(x_nom, y_nom, '*', 'color', colors(veh,:));
    else
      hn{veh}.XData = x_nom;
      hn{veh}.YData = y_nom;
    end
    
    % Plot vehicle number
    if isempty(ht{veh})
      ht{veh} = text(x_nom, y_nom, sprintf('%d', veh));
    else
      ht{veh}.Position = [x_nom y_nom];
    end
    
    % Plot position
    extraArgs.Color = colors(veh, :);
    extraArgs.ArrowLength = 3;
    extraArgs.MakerSize = 5;
    Q{veh}.plotPosition(extraArgs);
  end
end
end