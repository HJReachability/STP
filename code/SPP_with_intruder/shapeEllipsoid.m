function obs = shapeEllipsoid(grid, center, semi_axes)
% Generate an ellipse around center with given semi-axes

obs1 = 0;
obs2 = 0;
obs3 = 0;
for i = 1:grid.dim
  obs1 = obs1 + (grid.xs{i} - semi_axes(i)).^2 / semi_axes(i)^2;
  
  % Dealing with periodic conditions
  if isequal(grid.bdry{i}, @addGhostPeriodic)
    obs2 = obs2 + (grid.xs{i} - semi_axes(i) - 2*pi).^2 / semi_axes(i)^2;
    obs3 = obs3 + (grid.xs{i} - semi_axes(i) + 2*pi).^2 / semi_axes(i)^2;  
  else
    obs2 = obs2 + (grid.xs{i} - semi_axes(i)).^2 / semi_axes(i)^2;
    obs3 = obs3 + (grid.xs{i} - semi_axes(i)).^2 / semi_axes(i)^2;
  end
end

obs = min(obs1, obs2);
obs = min(obs, obs3);
end