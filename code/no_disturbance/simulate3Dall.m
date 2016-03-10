clearvars -except vnum
close all

%---------------------------------------------------------------------------
% Load game setup
Nx = 71;

% Create the computation grid.
g.dim = 3;
g.min = [  -1; -1; 0];
g.max = [ +1; +1; 2*pi];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx];

g = processGrid(g);

run('Game Data\OLGameCoop3D')

%---------------------------------------------------------------------------
% Load trajectories
trajs = cell(vnum,1);
taus = cell(vnum,1);
tMin = inf;
tMax = -inf;
for i = 1:vnum
    load(['coop3D_' num2str(i)],'traj','tau')
    trajs{i} = traj;
    taus{i} = tau;
    tMin = min([tMin min(tau)]);
    tMax = max([tMax max(tau)]);
end

dt = 0.01;

tMin = floor(tMin/dt)*dt;
tMax = ceil(tMax/dt)*dt;

t = tMin+dt:dt:tMax;

% Interpolate to get entire trajectory
com_t_traj = cell(vnum,1);
for i = 1:vnum
    com_t_traj{i} = tInterpolate(taus{i}, trajs{i}, t);
end


f1 = figure;
colors = {'b','r',[0 0.5 0], [0.75 0 0.75]};

% Plot trajectories
ht = cell(N,1);
hx = cell(N,1);
hxt = cell(N,1);
hc = cell(N,1);

plotnum = 0;
% i = 1, 33, 93, 148, 182 for paper
% for i = [33 93 148 182]
% for i = 1
for i = 1:length(t)
%     plotnum = plotnum + 1;
%     subplot(2,2,plotnum);
    
    % Plot static target set and obstacles
    [~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs_2D, [0 0], 'k','linewidth',2); hold on
    set(gca, 'fontsize',16)
    
    for j = 1:vnum
        [~, ht{j}] = contour(g2D.xs{1}, g2D.xs{2}, target_2D{j}, [0 0], 'color', colors{j},'linewidth',2);
    end
    
    for j = 1:vnum
        dirn = 0.2*[cos(com_t_traj{j}(i,3)) sin(com_t_traj{j}(i,3))];
        
        hx{j} = quiver(com_t_traj{j}(i,1), com_t_traj{j}(i,2), dirn(1), dirn(2), ...
            'maxheadsize', 50, 'marker', '*', 'color', colors{j});
%         hx{j} = plot(com_t_traj{j}(i,1), com_t_traj{j}(i,2),'.', 'color', colors{j}, 'markersize', 10);
        hxt{j} = plot(com_t_traj{j}(1:i,1), com_t_traj{j}(1:i,2), 'color', colors{j}, 'linestyle', ':');
        
        bubble = sqrt((g2D.xs{1} - com_t_traj{j}(i,1)).^2 + (g2D.xs{2} - com_t_traj{j}(i,2)).^2) - captureRadius;
        [~, hc{j}] = contour(g2D.xs{1}, g2D.xs{2}, bubble, [0 0], 'color', colors{j}, 'linestyle','--');
    end
    
    title(['t=' num2str(t(i))],'fontsize',14)
    xlim([-0.8 0.8])
    ylim([-0.8 0.8])
%     grid on
    axis square
    drawnow
    
    export_fig(['trajSim/' num2str(i)], '-png')
    if i == 182
        legend([ho ht{1} hx{2} hxt{3} hc{4}], {'Obstacle', 'Targets', 'Positions, Headings', 'Trajectories', 'Danger Zones'})
        set(legend,'location','bestoutside')
    end
    hold off
end

return

% Subplot spacing
subP_size = 0.325;
subP_xmin = 0.05;
subP_ymin = 0.1;
subP_xgap = 0.03;
subP_ygap = 0.15;

subP_pos = [subP_xmin               subP_ymin+subP_size+subP_ygap       subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin+subP_size+subP_ygap   subP_size subP_size;
    subP_xmin                       subP_ymin                      subP_size subP_size;
    subP_xmin+subP_size+subP_xgap   subP_ymin                      subP_size subP_size];

for j = 1:N % Fix defender position
    subplot(2,2,j)
    set(gca,'position',subP_pos(j,:))
end

pos = get(gcf,'position');
set(gcf,'position',[pos(1) pos(2) 800 600]);
set(legend,'units','pixels','position',[560 200 225 150])
legend('boxoff')