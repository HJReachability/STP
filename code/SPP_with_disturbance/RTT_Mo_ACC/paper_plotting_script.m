% Load the saved data
% load ex1

% %% Start working on Figure-1
% 
% % Create the grid
%     gMin = [-1; -1; -3*pi/2];
%     gMax = [1; 1; pi/2];
%     gN = [95; 95; 95];
%     g = createGrid(gMin, gMax, gN, 3);
% 
% %Assign the color
% fig_color = {'r', 'b', [0 0.5 0], 'k'};
% 
% % Assign the sub-plot axes to the vehicles
% % allVehicles = {Q1; Q2; Q3; Q4};
% vnum = 4;
% 
% figure,
% f = gcf;
% pos = get(f, 'position');
% set(f, 'position', [pos(1) pos(2) 560 300]);
% hold on;
% % Plot target sets, trajectory and some point in between
% tAbs = -1.1;
% small = 1e-4;
% for i=1:vnum
%     t = find(allVehicles{i}.tau < tAbs + small & allVehicles{i}.tau > tAbs - small);
%     if isempty(t)
%       t = length(allVehicles{i}.tau);
%     end
%     % Target set
%     [g2D, data2D] = proj2D(g, allVehicles{i}.target, [0 0 1]);
%     [~,h{i,1}] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', fig_color{i},'linewidth',2);
%     % State
%     plot(allVehicles{i}.x(1,t), allVehicles{i}.x(2,t), 'marker', '*', 'color', fig_color{i},'markersize',5);
%     % Arrow for the state
%     dirn = 0.2*[cos(allVehicles{i}.x(3,t)) sin(allVehicles{i}.x(3,t))];
%     h{i,2} = quiver(allVehicles{i}.x(1,t), allVehicles{i}.x(2,t), dirn(1), dirn(2), ...
%         'maxheadsize', 50, 'marker', '*', 'color', fig_color{i});
%     % Bubble around state
%     bubble =  sqrt((g.xs{1} - allVehicles{i}.x(1,t)).^2 +...
%         (g.xs{2} - allVehicles{i}.x(2,t)).^2) - 0.1;
%     [g2D, data2D] = proj2D(g, bubble, [0 0 1]);
%     [~,h{i,3}] =  contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', fig_color{i},'linestyle', '--');
%     % Also plot the full trajectory
%     h{i,4} = plot(allVehicles{i}.x(1,:), allVehicles{i}.x(2,:), 'color', fig_color{i},'linestyle', ':');
%     drawnow;
% end
% str = sprintf('t = %s',num2str(tAbs,'%.1f'));
% title(str);
% set(gca,'YTickMode','manual');
% set(gca,'YTick',[-0.5, 0, 0.5]);
% set(gca,'XTickMode','manual');
% set(gca,'XTick',[-0.5, 0, 0.5]);
% set(gca, 'Fontsize', 14);
% axis equal;
% box on;
% h_leg = legend([h{1,1}, h{1,2}, h{1,3}, h{1,4}],'Targets', 'Positions, Headings', 'Danger Zones', 'Trajectories', 'Location','EastOutside');
% set(h_leg, 'Fontsize', 14);
% hold off;

%% Start working on Figure-2
% Time and vehicle to plot
vehicle = 3;
t = [length(allVehicles{vehicle}.BRS1_tau)-50, 150, 100, 2];
tAbs = [allVehicles{vehicle}.BRS1_tau(t(1)), allVehicles{vehicle}.BRS1_tau(t(2)), allVehicles{vehicle}.BRS1_tau(t(3)), allVehicles{vehicle}.BRS1_tau(t(4))];
figure,
f = gcf;
pos = get(f, 'position');
set(f, 'position', [pos(1) pos(2) 560 600]);

% Plot the target sets and initial conditions
for j=1:length(t) % Loop over time steps
    h{j} = subplot(2, 2, j);
    hold on;
    for i=vehicle:vehicle % Plot for the vehicle
        % Target set
        [g2D, data2D] = proj2D(g, allVehicles{i}.target, [0 0 1]);
        [~,h1] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', fig_color{i},'linewidth',2);
        % Initial State
        plot(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), 'marker', '*', 'color', fig_color{i},'markersize',5);
        % Arrow for initial state
        dirn = 0.2*[cos(allVehicles{i}.x(3,1)) sin(allVehicles{i}.x(3,1))];
        h2 = quiver(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), dirn(1), dirn(2), ...
            'maxheadsize', 50, 'marker', '*', 'color', fig_color{i});
        % Reachable set
        [g2D, data2D] = proj2D(g, allVehicles{i}.BRS1(:,:,:,t(j)), [0 0 1], allVehicles{i}.x(3,1));
        [~, h3] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', fig_color{i},'linestyle', '-');
        % Overall obstacle shape
        vnum = i-1;
        obstacle = ones(g2D.N');
        for k=1:vnum
            index = find(allVehicles{k}.obs2D_tau < tAbs(j) + small & allVehicles{k}.obs2D_tau > tAbs(j) - small);
            if ~isempty(index)
              obstacle = shapeUnion(obstacle,allVehicles{k}.obs2D(:,:,index));
            end
        end
        if any(obstacle(:) < 0)
            [~, h4] = contour(g2D.xs{1},g2D.xs{2}, obstacle, [0 0], 'color', 'k');
        end
        drawnow;
        axis equal;
    end
    str = sprintf('t = %.2f',tAbs(j));
    title(str);
    set(gca,'YTickMode','manual');
    set(gca,'YTick',[-0.5, 0, 0.5]);
    set(gca,'XTickMode','manual');
    set(gca,'XTick',[-0.5, 0, 0.5]);
    set(gca, 'Fontsize', 14);
    axis equal;
    box on;
end
h_leg = legend([h1, h2, h3, h4], 'Targets', 'Initial pos. and heading', 'Reachable set', 'Obstacle', 'Location','SouthOutside');
set(h_leg, 'Fontsize', 14);
% Move the bottom two subplots a bit higher
p = get(h{3}, 'pos');
p(2) = 0.18;
set(h{3}, 'pos', p);

p = get(h{4}, 'pos');
p(2) = 0.18;
set(h{4}, 'pos', p);

% allVehicles{1}.tau = Q1.tau;
% allVehicles{2}.tau = Q2.tau;
% allVehicles{3}.tau = Q3.tau;
% allVehicles{4}.tau = Q4.tau;
% 
% allVehicles{1}.x = Q1.xhist;
% allVehicles{2}.x = Q2.xhist;
% allVehicles{3}.x = Q3.xhist;
% allVehicles{4}.x = Q4.xhist;

% allVehicles{1}.obs2D = Q1.obs2D;
% allVehicles{2}.obs2D = Q2.obs2D;
% allVehicles{3}.obs2D = Q3.obs2D;
% allVehicles{4}.obs2D = Q4.obs2D;
% 
% allVehicles{1}.obs2D_tau = Q1.obs2D_tau;
% allVehicles{2}.obs2D_tau = Q2.obs2D_tau;
% allVehicles{3}.obs2D_tau = Q3.obs2D_tau;
% allVehicles{4}.obs2D_tau = Q4.obs2D_tau;