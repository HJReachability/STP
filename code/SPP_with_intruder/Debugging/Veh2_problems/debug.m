%% Load the debugging data
% load 'SPPwIntruder_debug_FRSinclusionissue'

%% Grid
grid_min = [-1; -1; 0]; % Lower corner of computation domain
grid_max = [1; 1; 2*pi];    % Upper corner of computation domain
N = [101; 101; 101];         % Number of grid points per dimension
pdDims = 3;               % 3rd diemension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% Visualize the BRS with obstacles 
slice = pi;
veh = 2;

% First BRS and obstacles
figure,
for i=1:length(Q{veh}.tau_BRS1)
    % BRS
    data1 = Q{veh}.data_BRS1(:, :, :, i);
    [gProj, dataProj] = proj(g, data1, [0 0 1], slice);
    hT1 = visSetIm(gProj, dataProj, 'r', 0, [], false);
    hT1.LineWidth = 2;
    
    hold on;
    % obstacle
    data1 = unionObs(:, :, :, end+1-i);
    [gProj, dataProj] = proj(g, data1, [0 0 1], slice);
    hT2 = visSetIm(gProj, dataProj, 'b', 0, [], false);
    hT2.LineWidth = 2;
    
    hold off;
    pause();  
end