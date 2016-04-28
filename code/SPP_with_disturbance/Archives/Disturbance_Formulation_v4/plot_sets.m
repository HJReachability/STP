% % Vehicle 1
% index = 0;
% figure,
% [g2D, data2D] = proj2D(g, allVehicles{1}.reach(:,:,:,index+1), [0 0 1], allVehicles{1}.x(3,index+1));
% [~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');

% Vehicle 2
index = 0;
figure,
[g2D, data2D] = proj2D(g, allVehicles{2}.reach(:,:,:,index+1), [0 0 1], 75*pi/180);
[~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');
hold on;
[g2D, data2D] = proj2D(g, allVehicles{2}.cons_reach(:,:,:,index+1), [0 0 1], pi/2);
[~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'b', 'linestyle', '--');

index = 1;
figure,
[g2D, data2D] = proj2D(g, allVehicles{1}.collisionmat(:,:,:,index+1), [0 0 1], pi/2);
[~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');

index = 1;
figure,
[g2D, data2D] = proj2D(g, temp1(:,:,:,index+1), [0 0 1], pi/2);
[~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');

% 
% figure,
% [~,h3] = contour(allVehicles{2}.plot_grid{2}.xs{1},    allVehicles{2}.plot_grid{2}.xs{2},...     
% allVehicles{2}.plot_data{2}, [0 0], 'color', 'k', 'linestyle', '--');
% hold on;
% [~,h4] = contour(allVehicles{2}.plot_grid{3}.xs{1},    allVehicles{2}.plot_grid{3}.xs{2},...     
% allVehicles{2}.plot_data{3}, [0 0], 'color', 'g', 'linestyle', '--');

% Siulate the trajectory of vehicle 2
% figure,
% for index=0:60
% [g2D, data2D] = proj2D(g, allVehicles{2}.reach(:,:,:,index+1), [0 0 1], allVehicles{2}.x(3,index+1));
% [~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'r', 'linestyle', '--');
% hold on;
% [g2D, data2D] = proj2D(g, allVehicles{2}.cons_reach(:,:,:,index+1), [0 0 1], allVehicles{2}.x(3,index+1));
% [~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'b', 'linestyle', '--');
% x_next = allVehicles{2}.x(:,index+1);
% plot(x_next(1), x_next(2), 'marker', 'o', 'color', allVehicles{2}.fig_color,'markersize',5);
% delete(allVehicles{2}.quiver_hand);
% dirn = 0.2*[cos(x_next(3)) sin(x_next(3))];
% allVehicles{2}.quiver_hand = quiver(x_next(1),x_next(2), dirn(1), dirn(2), ...
%         'maxheadsize', 50, 'marker', '*', 'color', allVehicles{2}.fig_color);
% drawnow;
% pause;
% delete(h1);
% delete(h2);
% end

simulate_trajectory(g, allVehicles{2},0.6)