clear all;
figure

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

% subplot(2,2,1)
figure
% contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
% axis square
% set(gca, 'fontsize', 16)
% 
% contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', 'r', 'linewidth', 2);
% plot(x2_init{1}(1), x2_init{1}(2), 'r.')

% i = 450, 290, 240 1
plotnum = 0;
for i = length(tau):-7:1

% for i = [450 290 240 1]
    if i<length(tau)
        delete(hr); 
        delete(hc);
    end
    
    plotnum = plotnum+1;
%     subplot(2,2,plotnum)
    [~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k', 'linewidth', 2); hold on
    axis square
    set(gca, 'fontsize', 16)

    [~, ht] = contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'color', 'r', 'linewidth', 2);
    hx = plot(x2_init{1}(1), x2_init{1}(2), 'r*') ;   
    
    [~, hr] = contour(g2D.xs{1}, g2D.xs{2}, reach(:,:,i), [0 0], 'r');
    [~, hc] = contour(g2D.xs{1}, g2D.xs{2}, obstacle(:,:,i), [0 0], 'k','linestyle','--');
    
%     title(num2str(i))
    title(['t=' num2str(tau(i), '%4.2f')])
    drawnow
    export_fig(['kinsimVIReach/' num2str(plotnum)], '-png')
end

legend([ho ht hx hr hc], {'Obstacle', 'Target',  ...
    'Position', 'Reach-avoid set (VI)', 'Danger zone'})
set(legend,'location','bestoutside')

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
set(gcf,'position',[pos(1) pos(2) 800 600]);
set(legend,'units','pixels','position',[560 200 225 150])
legend('boxoff')