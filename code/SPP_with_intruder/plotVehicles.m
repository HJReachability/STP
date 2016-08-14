function plotVehicles(Q, tInd, schemeData, hc, ho, colors, capture_radius)
% plotVehicles(Q, hc, ho, colors, capture_radius)
%     Updates the plot in the SPP simulation

for veh = 1:length(Q)
  % Plot capture radius
  if isempty(hc{veh})
    hc{veh} = plotDisk( ...
      Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
  else
    [~, hc{veh}.XData, hc{veh}.YData] = plotDisk( ...
      Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
  end
  
  % Plot induced obstacle for vehicles 1 to 3
  if veh < length(Q)
    [g2D, data2D] = ...
      proj(schemeData.grid, Q{veh}.data.cylObs3D(:,:,:,tInd), [0 0 1]);
    if isempty(ho{veh})
      ho{veh} = visSetIm(g2D, data2D, colors(veh, :));
      ho{veh}.LineStyle = '--';
    else
      ho{veh}.ZData = data2D;
    end
  end
  
  % Plot position
  Q{veh}.plotPosition(colors(veh, :));
end
end