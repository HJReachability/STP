function plotSetup(obj, save_png)
% plotSetup(obj)
%   Plots the problem setup

if nargin < 2
  save_png = true;
end

figure

% Map
I = imread(obj.mapFile);
I = I(1:800, 701:1500, :);
I = flip(I, 1);
imshow(I, 'InitialMagnification', 75, 'XData', [0 500], 'YData', [0 500]);

hold on

% Initial states
k = 5;
for i = 1:length(obj.initStates)
  quiver(obj.initStates{i}(1), obj.initStates{i}(2), ...
    k*cos(obj.initStates{i}(3)), k*sin(obj.initStates{i}(3)), '*', ...
    'MarkerSize', 5*k);
end

% Targets
for i = 1:length(obj.targetCenters)
  plotDisk(obj.targetCenters{i}, obj.targetR, 'linewidth', 3);
end

% Obstacles
h = visSetIm(obj.g2D, obj.staticObs, 'k');
h.LineWidth = 3;

% % Grid
% plot(g2D.xs{1}(:), g2D.xs{2}(:), 'r.')

axis xy
axis on
grid on

if save_png
  export_fig(sprintf('%s/setup', obj.folder), '-png', 'm2')
end

end