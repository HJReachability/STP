% -------------------------
% Reachable set at final tf
clear all; close all;

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

% figure;
% colors = {'b','r',[0 0.5 0], [0.75 0 0.75]};
% for vnum = 1:N
%     load(['coop3D_' num2str(vnum)],'reach','g','tau','obstacle')
%     
%     subplot(2,2,vnum)
%     [~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs_2D, [0 0], 'k', 'linewidth', 2); hold on
%     set(gca, 'fontsize',16)
%     
%     [~, ht] = contour(g2D.xs{1}, g2D.xs{2}, target_2D{vnum}, [0 0], ...
%         'color', colors{vnum}, 'linewidth', 2); 
%     
%     [~, reach2D] = proj2D(g, [0 0 1], N2D, reach(:,:,:,1), x_init{vnum}(3));
%     [~, hr] = contour(g2D.xs{1}, g2D.xs{2}, reach2D, [0 0], 'color', colors{vnum});
% 
%     [~, coll2D] = proj2D(g, [0 0 1], N2D, obstacle(:,:,:,1), x_init{vnum}(3));
%     [~, hc] = contour(g2D.xs{1}, g2D.xs{2}, coll2D, [0 0], 'k', 'linestyle','--');
%    
%     dirn = 0.125*[cos(x_init{vnum}(3)) sin(x_init{vnum}(3))];
%     hx = quiver(x_init{vnum}(1), x_init{vnum}(2), dirn(1), dirn(2), ...
%         'maxheadsize', 15, 'marker', '.', 'color', colors{vnum});
%     
%     if vnum == 4
%         legend([ho ht hx hr hc], {'Obstacle', 'Targets', ...
%             'Initial pos. and heading', 'Reach-avoid set', 'Danger zones'})
%     end
%     
%     title(['Vehicle ' num2str(vnum)], 'fontsize', 14)
%     axis square
%     drawnow;
% end
% return
% -------------------------
% Reachable set of vehicle 3 at various times
figure;
colors = {'b','r',[0 0.5 0], [0.75 0 0.75]};
vnum = 4
load(['coop3D_' num2str(vnum)],'reach','g','tau','obstacle')



% i = 0, 100, 200, 310
plotnum = 0;
% for i = [310 211 111 1]
for i = length(tau):-1:1
%     plotnum = plotnum + 1;
%     subplot(2,2,plotnum)
    
    [~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs_2D, [0 0], 'k', 'linewidth', 2); hold on
    set(gca, 'fontsize',16)
    
    [~, ht] = contour(g2D.xs{1}, g2D.xs{2}, target_2D{vnum}, [0 0], ...
        'color', colors{vnum}, 'linewidth', 2);
    
    dirn = 0.125*[cos(x_init{vnum}(3)) sin(x_init{vnum}(3))];
    hx = quiver(x_init{vnum}(1), x_init{vnum}(2), dirn(1), dirn(2), ...
        'maxheadsize', 15, 'marker', '.', 'color', colors{vnum});

    [~, reach2D] = proj2D(g, [0 0 1], N2D, reach(:,:,:,i), x_init{vnum}(3));
    [~, hr] = contour(g2D.xs{1}, g2D.xs{2}, reach2D, [0 0], 'color', colors{vnum});

    [~, coll2D] = proj2D(g, [0 0 1], N2D, obstacle(:,:,:,i), x_init{vnum}(3));
    [~, hc] = contour(g2D.xs{1}, g2D.xs{2}, coll2D, [0 0], 'k', 'linestyle','--');
   
    if i == 1
        legend([ho ht hx hr hc], {'Obstacle', 'Targets', ...
            'Initial pos. and heading', 'Reach-avoid set', 'Danger zones'})
    end
    
%     if i < length(tau)
        title(['t=' num2str(tau( i ), '%4.2f' ) ], 'fontsize', 14)
%     else
%         title('t=0.4', 'fontsize', 14)
%     end
    axis square
    drawnow;
    
    export_fig(['reachSim' num2str(vnum) '/' num2str(i)], '-png')
    
    hold off
%     if i<size(reach,4)
%         delete(hr)
%         delete(hc)
%     end
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

for j = 1:4 % Fix defender position
    subplot(2,2,j)
    set(gca,'position',subP_pos(j,:))
end

pos = get(gcf,'position');
set(gcf,'position',[pos(1) pos(2) 600 800]);
set(legend,'units','pixels','position',[560 200 225 150])
