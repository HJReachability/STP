close all; clear; 

figName = 'rtt_rs3.fig';
plotsToKeep = [1, 3, 4]; 
titlestr = {'t = -0.50', 't = -2.59', 't = -3.57'};
% Open the setup fig file
h1 = openfig(figName,'reuse'); % open figure
% Extract the data for this figure
h = gcf; 
axesObjs = get(h, 'Children');  %axes handles

% Create new figure
f = figure; 
pos = get(f, 'position');
set(f, 'position', [pos(1) pos(2) 560 150]);

for i=1:3
  %create and get handle to the subplot axes
  s{i} = subplot(1,3,i);
  % get handle to all the children in the figure
  child{i} = axesObjs(6-plotsToKeep(i));  % get handle to axes of figure
  % copy children to new parent axes i.e. the subplot axes
  copyobj(child{i}.Children,s{i});
  % Set the ticks
  set(gca,'YTickMode','manual');
  set(gca,'YTick',[-0.5, 0, 0.5]);
  set(gca,'XTickMode','manual');
  set(gca,'XTick',[-0.5, 0, 0.5]);
  % set x and y limits
  % Set Fontsize and axes
  set(gca, 'Fontsize', 14);
  axis equal;
  box on;
  set(findall(gca, 'Type', 'Line'),'LineWidth',2.5);
  title(titlestr(i));
  if i==2
    obj1 = findobj(gca, 'Type', 'quiver');
    obj2 = findobj(gca, 'Type', 'contour');
    legend([obj1; obj2], 'Initial pos. and heading', 'Obstacle', 'BRS', 'Targets');
  end
end
