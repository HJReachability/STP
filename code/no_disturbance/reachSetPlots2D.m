clear all;
figure

% ------ VI TRAJECTORY --------
subplot(2,2,1)
load('coop2D.mat')
Nx = 45;
% Nx = 31;

% Create the computation grid.
clear g
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);
run('Game Data\OLGameCoop2D')
contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
set(gca, 'fontsize', 16)
contour(g2D.xs{1}, g2D.xs{2}, target12D, [0 0], 'color', 'b', 'linewidth', 2);
contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', 'r', 'linewidth', 2);
htraj1 = plot(traj1(:,1), traj1(:,2), 'b:');
htraj2 = plot(traj2(:,1), traj2(:,2), 'r:');
plot(traj1(end,1), traj1(end,2), 'b.')
plot(traj2(end,1), traj2(end,2), 'r.')
axis square
title('Trajectories (VI)', 'fontsize', 14)

% --------- HJI TRAJECTORY ---------
subplot(2,2,2)
load(['F:\coop2D\coop4D_80'])
Nx = 45;
% Nx = 31;

% Create the computation grid.
clear g
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);
run('Game Data\OLGameCoop2D')
contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
set(gca, 'fontsize', 16)
contour(g2D.xs{1}, g2D.xs{2}, target12D, [0 0], 'color', 'b', 'linewidth', 2);
contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', 'r', 'linewidth', 2);
plot(traj1(:,1), traj1(:,2), 'b:')
plot(traj2(:,1), traj2(:,2), 'r:')
plot(traj1(end,1), traj1(end,2), 'b.')
plot(traj2(end,1), traj2(end,2), 'r.')
axis square
title('Trajectories (HJI)', 'fontsize', 14)

% ----------- REACHABLE SETS ------
subplot(2,2,3)

load(['F:\coop2D\coop4D_80'])
run('Game Data\OLGameCoop2D')
[~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
set(gca, 'fontsize', 16)
[~, ht] = contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', 'r', 'linewidth', 2);

xs = x1_init{1};
bubble = sqrt((g2D.xs{1} - xs(1)).^2 + (g2D.xs{2} - xs(2)).^2) - captureRadius;
[~, hc] = contour(g2D.xs{1}, g2D.xs{2}, bubble, [0 0], 'k', 'linestyle', '--'); 

[~, data2D] = proj2D(g, [1 1 0 0], N2D, data, xs);
[~, hrh] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'b');

hx = plot(x2_init{1}(1), x2_init{1}(2), 'r.');

axis square

load('coop2D.mat')
% % figure
% contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
% contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', [0 0.5 0], 'linewidth', 2);
% 
% bubble = sqrt((g2D.xs{1} - xs(1)).^2 + (g2D.xs{2} - xs(2)).^2) - captureRadius;
% contour(g2D.xs{1}, g2D.xs{2}, bubble, [0 0], 'k', 'linestyle', '--'); 

[~, hrv] = contour(g2D.xs{1}, g2D.xs{2}, reach(:,:,1), [0 0], 'r');

% plot(x2_init{1}(1), x2_init{1}(2), 'r.')

title('Reach-avoid sets at t=t^i_2','fontsize',14)


legend([ho ht htraj1 hx hrh hrv hc], {'Obstacle', 'Target', 'Trajectory' ...
    'Position', 'Reach-avoid set (HJI)', 'Reach-avoid set (VI)', 'Danger zone'})
set(legend,'location','bestoutside')
axis square

% ----- PLOT POSITION AND SIZE

subP_size = 0.325;
subP_xmin = 0.05;
subP_ymin = 0.1;
subP_xgap = 0.03;
subP_ygap = 0.15;

subP_pos = [subP_xmin               subP_ymin+subP_size+subP_ygap       subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin+subP_size+subP_ygap   subP_size subP_size;
    subP_xmin                       subP_ymin                      subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin                      subP_size subP_size];

for j = 1:3
    subplot(2,2,j)
    set(gca,'position',subP_pos(j,:))
end

pos = get(gcf,'position');
set(gcf,'position',[pos(1) pos(2) 675 600]);
set(legend,'units','pixels','position',[300 100 225 150])
legend('boxoff')
