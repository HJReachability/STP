function plotSPPP(mapFile, targetCentersSet, targetR, g2D, obs2D, initState)


figure
% Map
I = imread(mapFile);
I = I(1:800, 701:1500, :);
I = flip(I, 1);
imshow(I, 'InitialMagnification', 75, 'XData', [0 500], 'YData', [0 500]);

hold on

% Initial states
k = 100;
quiver(initState(1), initState(2), k*cos(initState(3)), ...
  k*sin(initState(3)), '*', 'MarkerSize', 25);
hold on

% Targets
for i = 1:length(targetCentersSet)
  plotDisk(targetCentersSet{i}, targetR, 'linewidth', 3);
end

% Obstacles
h = visSetIm(g2D, obs2D, 'k');
h.LineWidth = 3;

% % Grid
% plot(g2D.xs{1}(:), g2D.xs{2}(:), 'r.')

axis xy
axis on
grid on
end