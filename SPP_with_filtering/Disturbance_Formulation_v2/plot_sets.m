% Vehicle 1
index = 0;
figure,
[g2D, data2D] = proj2D(g, allVehicles{1}.reach(:,:,:,index+1), [0 0 1], allVehicles{1}.x(3,index+1));
[~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');

% Vehicle 2
index = 0;
figure,
[g2D, data2D] = proj2D(g, allVehicles{2}.reach(:,:,:,index+1), [0 0 1], allVehicles{2}.x(3,index+1));
[~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');
hold on;
[g2D, data2D] = proj2D(g, allVehicles{2}.cons_reach(:,:,:,index+1), [0 0 1], allVehicles{2}.x(3,index+1));
[~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'b', 'linestyle', '--');

[~,h3] = contour(allVehicles{2}.plot_grid{2}.xs{1},    allVehicles{2}.plot_grid{2}.xs{2},...     
allVehicles{2}.plot_data{2}, [0 0], 'color', 'k', 'linestyle', '--');
hold on;
[~,h4] = contour(allVehicles{2}.plot_grid{3}.xs{1},    allVehicles{2}.plot_grid{3}.xs{2},...     
allVehicles{2}.plot_data{3}, [0 0], 'color', 'g', 'linestyle', '--');

