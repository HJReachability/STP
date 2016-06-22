%% Load the debugging data
% load 'SPPwIntruder_debug_FRSinclusionissue'

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% Visualize a slice
slice = 3.45;
figure,
% First BRS
data1 = Q{2}.data_BRS1(:, :, :, end);
[gProj, dataProj] = proj(g, data1, [0 0 1], slice);
hT1 = visSetIm(gProj, dataProj, 'r', 0, [], false);
hT1.LineWidth = 2;

hold on;
% FRS
data2 = dataFRS(:, :, :, end);
[gProj, dataProj] = proj(g, data2, [0 0 1],slice);
hT2 = visSetIm(gProj, dataProj, 'b', 0, [], false);
hT2.LineWidth = 2;

% Second BRS
data3 = Q{2}.data_BRS2(:, :, :, end);
[gProj, dataProj] = proj(g, data3, [0 0 1], slice);
hT3 = visSetIm(gProj, dataProj, 'k', 0, [], false);
hT3.LineWidth = 2;

legend('BRS1', 'FRS', 'BRS2');

%% Visualize a set
figure,
% First BRS
data1 = Q{1}.data_BRS1(:, :, :, end);
hT1 = visSetIm(g, data1, 'r', 0, [], false);

hold on;
% FRS
data2 = dataFRS(:, :, :, end);
hT2 = visSetIm(g, data2, 'b', 0, [], false);

% Second BRS
data3 = Q{1}.data_BRS2(:, :, :, end);
hT3 = visSetIm(g, data3, 'k', 0, [], false);

hT3.FaceAlpha = 0.3;
hT2.FaceAlpha = 0.6;

legend('BRS1', 'FRS', 'BRS2');