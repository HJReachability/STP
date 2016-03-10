%---------------------------------------------------------------------------
% Load game info and 4D HJI Data
clear all;
% close all
% mex mexEikonalFMM.cpp

load('F:\coop2D\coop4D_80','tau','g')
load('F:\coop2D\coop4D_1','data')
% load('coop2D')

%---------------------------------------------------------------------------
% Grid of joint space
Nx = 45;
% Nx = 31;

% Create the computation grid.
clear g
g.dim = 4;
g.min = [  -1; -1; -1; -1];
g.max = [ +1; +1; +1; +1 ];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate; @addGhostExtrapolate };
g.N = [ Nx; Nx; Nx; Nx ];

g = processGrid(g);

run('Game Data\OLGameCoop2D')


% %---------------------------------------------------------------------------
% % Shortest paths
u1 = compute_value(g2D, target12D,velocity1, obs2D);
P1 = extractCostates(g2D, u1);

%---------------------------------------------------------------------------
% Integrate trajectory
% dt = 0.01;
% Tf = 2.5;
x1 = x1_init{1};
x2 = x2_init{1};
% x1 = [0.8 0.4];
% x2 = [0.6 0.3];


f1 = figure;
contour(g2D.xs{1}, g2D.xs{2}, target12D, [0 0], 'b','linewidth',2); hold on
contour(g2D.xs{1}, g2D.xs{2}, target22D, [0 0], 'r','linewidth',2);
contour(g2D.xs{1}, g2D.xs{2}, obs2D, [0 0], 'k','linewidth',2);
hx1 = plot(x1(1),x1(2),'b*','markersize',10);
hx2 = plot(x2(1),x2(2),'r*','markersize',10);
title('t=0')
axis square
drawnow;

traj1 = x1;
traj2 = x2;
hxt1 = plot(traj1(:,1), traj1(:,2), 'b:');
hxt2 = plot(traj2(:,1), traj2(:,2), 'r:');

% keyboard
for i = 2:length(tau)
  %     if i<length(tau)
  delete(hx1)
  delete(hx2)
  delete(hxt1)
  delete(hxt2)
  %     end
  
  % ---- 4D simulation --
  tic;load(['F:\coop2D\coop4D_' num2str(length(tau)-i+2)], 'P');toc
  %     tic;load(['F:\coop2D\coop4D_' num2str(i)]);toc
  [dir1, dir2] = HJIDirectionCoop4D(g,P,x1,x2);
  % ----------------------
  
  %     % ---- 2D + 2D simulation --
  %     dir1 = shortestPathDirectionP(g2D, P1, x1);
  %
  %     P2 = extractCostates(g2D,reach(:,:,i));
  %     p2 = calculateCostate(g2D,P2,x2);
  %     dir2 = -p2/norm(p2);
  %     % -------------------------
  
  if norm(x1-x2) <= captureRadius
    disp('Collision!')
    return
  end
  
  if eval_u(g, target4D,[x1 x2]) < 0
    disp('Destinations reached!')
    return
  end
  
  dt = tau(i) - tau(i-1);
  x1 = x1 + dir1*velocity1*dt;
  x2 = x2 + dir2*velocity2*dt;
  traj1 = cat(1, traj1, x1);
  traj2 = cat(1, traj2, x2);
  
  hx1 = plot(x1(1),x1(2),'b*','markersize',10);
  hx2 = plot(x2(1),x2(2),'r*','markersize',10);
  
  hxt1 = plot(traj1(:,1), traj1(:,2), 'b:');
  hxt2 = plot(traj2(:,1), traj2(:,2), 'r:');
  
  title(['t=' num2str(tau(i) - min(tau), '%4.3f')])
  drawnow;
  
  %     if ~mod(i,2)
  % %         export_fig(['kinsimVI/' num2str(i)], '-png')
  %
  %     end
  
  export_fig(['kinsimPDE/' num2str(i)], '-png')
end
