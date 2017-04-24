function f = plotSetup(obj, setup_name, save_png)
% plotSetup(obj)
%   Plots the problem setup

if nargin < 3
  save_png = true;
end

if nargout > 0
  f = figure;
else
  figure;
end

%% Map
I = imread(obj.mapFile);
switch setup_name
  case 'SF'
    I = I(1:800, 701:1500, :);
    I = flip(I, 1);
    imshow(I, 'InitialMagnification', 75, 'XData', [0 500], 'YData', [0 500]);
    
    colors = lines(length(obj.targetCentersSet));
    ArrowLength = 25;
    MarkerSize = 10;
    
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
    ArrowLength*cos(obj.initStates{i}(3)), ...
    ArrowLength*sin(obj.initStates{i}(3)), 's', ...
    'MarkerSize', MarkerSize, 'MaxHeadSize', 10, 'MarkerFaceColor', 'b', ...
    'MarkerEdgeColor', 'b');
end

%% Targets
for i = 1:length(obj.targetCentersSet)
  plotDisk(obj.targetCentersSet{i}, obj.targetR, 'linewidth', 3, 'color', ...
    colors(i,:));
end

%% Obstacles
if ~isempty(obj.staticObs)
  % Remove border obstacle
  obs_no_border = obj.staticObs;
  obs_no_border(1:5, :) = 10;
  obs_no_border(end-4:end, :) = 10;
  obs_no_border(:,end-4:end) = 10;
  obs_no_border(:,1:5) = 10;
  h = visSetIm(obj.g2D, obs_no_border, 'k');
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