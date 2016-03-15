% Load the saved data
% load ex1

%% Start working on Figure-1

% Assign the color
fig_color = {'r', 'b', [0 0.5 0], 'k'};

% Assign the sub-plot axes to the vehicles
vnum = 4;
for i=1:vnum
    allVehicles{i}.fig_color = fig_color{i};
end

figure,
f = gcf;
pos = get(f, 'position');
set(f, 'position', [pos(1) pos(2) 560 300]);
% subplot(1, 2, 1);
% hold on;
% Plot the target sets and initial conditions
% for i=1:vnum
%     % Target set
%     [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1]);
%     [~,h{i,1}] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
%     % Initial State
%     h{i,2} = plot(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), 'marker', '*', 'color', allVehicles{i}.fig_color,'markersize',5);
%     % Arrow for initial state
%     dirn = 0.2*[cos(allVehicles{i}.x(3,1)) sin(allVehicles{i}.x(3,1))];
%     h{i,3} = quiver(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), dirn(1), dirn(2), ...
%         'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
%     % Initial bubble
%     bubble =  sqrt((g.xs{1} - allVehicles{i}.x(1,1)).^2 +...
%         (g.xs{2} - allVehicles{i}.x(2,1)).^2) - 0.1;
%     [g2D, data2D] = proj2D(g, bubble, [0 0 1]);
%     [~, h{i,4}] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linestyle', '--');
%     drawnow;
% end
% title('Initial Setup');
% set(gca,'YTickMode','manual');
% set(gca,'YTick',[-0.5, 0, 0.5]);
% set(gca,'XTickMode','manual');
% set(gca,'XTick',[-0.5, 0, 0.5]);
% set(gca, 'Fontsize', 14);
% %legend(h{1,1}, 'Vehicle 1');
% h_leg = legend([h{1,1}, h{2,1}, h{3,1}, h{4,1}], 'Vehicle 1' , 'Vehicle 2', 'Vehicle 3',...
%     'Vehicle 4', 'Location','SouthOutside');
% set(h_leg, 'Fontsize', 14);
% axis equal;
% box on;
% 
% subplot(1, 2, 2);
hold on;
%Plot target sets, trajectory and some point in between
t = 200;
for i=1:vnum
    % Target set
    [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1]);
    [~,h{i,1}] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
    % State
    plot(allVehicles{i}.x(1,t), allVehicles{i}.x(2,t), 'marker', '*', 'color', allVehicles{i}.fig_color,'markersize',5);
    % Arrow for the state
    dirn = 0.2*[cos(allVehicles{i}.x(3,t)) sin(allVehicles{i}.x(3,t))];
    h{i,2} = quiver(allVehicles{i}.x(1,t), allVehicles{i}.x(2,t), dirn(1), dirn(2), ...
        'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
    % Bubble around state
    bubble =  sqrt((g.xs{1} - allVehicles{i}.x(1,t)).^2 +...
        (g.xs{2} - allVehicles{i}.x(2,t)).^2) - 0.1;
    [g2D, data2D] = proj2D(g, bubble, [0 0 1]);
    [~,h{i,3}] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linestyle', '--');
    % Also plot the full trajectory
    h{i,4}  = plot(allVehicles{i}.x(1,:), allVehicles{i}.x(2,:), 'color', allVehicles{i}.fig_color,'linestyle', ':');
    drawnow;
end
str = sprintf('t = %s',num2str(-t_end + t*0.01,'%.1f'));
title(str);
set(gca,'YTickMode','manual');
set(gca,'YTick',[-0.5, 0, 0.5]);
set(gca,'XTickMode','manual');
set(gca,'XTick',[-0.5, 0, 0.5]);
set(gca, 'Fontsize', 14);
axis equal;
box on;
h_leg = legend([h{1,1}, h{1,2}, h{1,3}, h{1,4}], 'Targets', 'Positions, Headings', 'Danger Zones', 'Trajectories', 'Location','EastOutside');
set(h_leg, 'Fontsize', 14);
hold off;




% %% Start working on Figure-2
% % Time and vehicle to plot
% t = [250, 200, 150, 23] + 50;
% vehicle = 3;
% figure,
% f = gcf;
% pos = get(f, 'position');
% set(f, 'position', [pos(1) pos(2) 560 600]);
% 
% % Plot the target sets and initial conditions
% for j=1:length(t) % Loop over time steps
%     h{j} = subplot(2, 2, j);
%     hold on;
%     for i=vehicle:vehicle % Plot for the vehicle
%         % Target set
%         [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1]);
%          [~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
%         % Initial State
%         plot(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), 'marker', '*', 'color', allVehicles{i}.fig_color,'markersize',5);
%         % Arrow for initial state
%         dirn = 0.2*[cos(allVehicles{i}.x(3,1)) sin(allVehicles{i}.x(3,1))];
%         h2 = quiver(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), dirn(1), dirn(2), ...
%             'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
%         % Reachable set
%         [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,t(j)), [0 0 1], allVehicles{i}.x(3,1));
%         [~, h3] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linestyle', '-');
%         % Overall obstacle shape
%         vnum = i-1;
%         obstacle = 1e6*ones(g.shape);
%         for k=1:vnum
%             obstacle = shapeUnion(obstacle,allVehicles{k}.collisionmat(:,:,:,t(j)));
%         end
%         [g2D, data2D] = proj2D(g, obstacle, [0 0 1]);
%         dataout = addCRadius(g2D, data2D, allVehicles{i}.capture_radius);
%         if any(dataout(:) < 0)
%         [~, h4] = contour(g2D.xs{1},g2D.xs{2}, dataout, [0 0], 'color', 'k');
%         end
%         drawnow;
%         axis equal;
%     end
%     str = sprintf('t = %s',num2str(-t_end + t(j)*0.01,'%.2f'));
%     title(str, 'Fontsize', 14);
%     set(gca,'YTickMode','manual');
%     set(gca,'YTick',[-0.5, 0, 0.5]);
%     set(gca,'XTickMode','manual');
%     set(gca,'XTick',[-0.5, 0, 0.5]);
%     set(gca, 'Fontsize', 14);
%     axis equal;
%     box on;
% end
% h_leg = legend([h1, h2, h3, h4], 'Targets', 'Initial pos. and heading', 'Reachable set', 'Obstacle', 'Location','SouthOutside');
% set(h_leg, 'Fontsize', 14);
% % Move the bottom two subplots a bit higher
% p = get(h{3}, 'pos');
% p(2) = 0.18;
% set(h{3}, 'pos', p);
% 
% p = get(h{4}, 'pos');
% p(2) = 0.18;
% set(h{4}, 'pos', p);