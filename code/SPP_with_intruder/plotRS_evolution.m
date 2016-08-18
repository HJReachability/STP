function plotRS_evolution(Q, veh, schemeData, obs_field, RS_field, plotDim, ...
  save_png, save_fig)
% plotBRS1_evolution(Q, veh, schemeData, obs_field)
%     Plots the backward time-evolution of BRS1 and and the induced obstacles
%     given in the field obs_field

if nargin < 5
  RS_field = 'BRS1';
end

if nargin < 6
  plotDim = 2;
end

if plotDim ~= 2 && plotDim ~= 3
  error('plotDim must be 2 or 3!')
end

if nargin < 7
  save_png = true;
end

if nargin < 8
  save_fig = false;
end

if save_png || save_fig
  % For saving graphics
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
end

theta0 = Q{veh}.x(3);

f = figure;
for i = 1:length(Q{veh}.data.(sprintf('%s_tau', RS_field)))
  % Plot BRS
  if plotDim == 2
    [g, RS] = ...
      proj(schemeData.grid, Q{veh}.data.(RS_field)(:,:,:,i), [0 0 1], theta0);
  else
    g = schemeData.grid;
    RS = Q{veh}.data.(RS_field)(:,:,:,i);
  end
  visSetIm(g, RS);
  hold on
  
  % Plot obstacle at each time step
  small = 1e-4;
  t = Q{veh}.data.(sprintf('%s_tau', RS_field))(i);
  
  for j = 1:veh-1
    % Determine the corresponding time index in the obstacles
    oInd = find(Q{j}.data.(sprintf('%s_tau', obs_field)) < t+small & ...
      Q{j}.data.(sprintf('%s_tau', obs_field)) > t-small);
    
    % If a time index is found, plot the obstacle at that time step
    if ~isempty(oInd)
      if plotDim == 2
        [~, Obs] = proj(schemeData.grid, ...
          Q{j}.data.(obs_field)(:,:,:,oInd), [0 0 1], theta0);
      else
        Obs = Q{j}.data.(obs_field)(:,:,:,oInd);
      end
      visSetIm(g, Obs, 'k');
    end
  end
  title(sprintf('t = %f', Q{veh}.data.(sprintf('%s_tau', RS_field))(i)))
  drawnow;
  
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
  
  clf
end