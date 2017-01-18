function plotSetup(obj, setup_name, save_png)
% plotSetup(obj)
%   Plots the problem setup

if nargin < 3
  save_png = true;
end

figure

%% Map
I = imread(obj.mapFile);
switch setup_name
  case 'SF'
    I = I(1:800, 701:1500, :);
    I = flip(I, 1);
    imshow(I, 'InitialMagnification', 75, 'XData', [0 500], 'YData', [0 500]);
    
    ArrowLength = 25;
    MarkerSize = 15;
    
  case 'Bay_Area'
    I = I(155:915, 1120:1770, :);
    I = flip(I, 1);
    imshow(I, 'InitialMagnification', 100, 'XData', [-125 1500], 'YData', ...
      [0 1900]);
    
    ArrowLength = 300;
    MarkerSize = 25;    
end

hold on
%% Initial states

for i = 1:length(obj.initStates)
  quiver(obj.initStates{i}(1), obj.initStates{i}(2), ...
    ArrowLength*cos(obj.initStates{i}(3)), ArrowLength*sin(obj.initStates{i}(3)), '*', ...
    'MarkerSize', MarkerSize);
end

%% Targets
for i = 1:length(obj.targetCenters)
  plotDisk(obj.targetCenters{i}, obj.targetR, 'linewidth', 3);
end

%% Obstacles
if ~isempty(obj.staticObs)
  h = visSetIm(obj.g2D, obj.staticObs, 'k');
  h.LineWidth = 3;
end

% % Grid
% plot(obj.g2D.xs{1}(:), obj.g2D.xs{2}(:), 'r.')
% grid_bounds = shapeRectangleByCorners(obj.g2D, obj.g2D.min+1, obj.g2D.max-1);
% visSetIm(obj.g2D, grid_bounds)

axis xy
axis on
grid on

if save_png
  export_fig(sprintf('%s/setup', obj.folder), '-png', '-m2')
end

end