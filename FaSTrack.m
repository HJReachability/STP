function FaSTrack()
% FaSTrack

%% Setup
gMin = [0 0];
gMax = [50 50];
gN = [20 20];

g = createGrid(gMin, gMax, gN);
%temp_g = createGrid([-20 -20], [20 20], [15 15]);  
%obs = shapeRectangleByCorners(g, [30; 25], [35; 30]);
obs = shapeRectangleByCorners(g, [30; 25], [40; 35]);

vxmax = 1.5;
vymax = 1;
P = Plane2D([47; 20], vxmax, vymax);
wmax = 2*pi;
aRange = [-1.5, 1.5];
dMax = 2;
dims = 1:4;
Q = Plane4D([47; 20; 0; 0], wmax, aRange, dMax, dims);

target = [20; 45];
targetLength = 1;

%% Computing Tracking Error Bound (TEB)
TEBgN = [55; 55; 25; 25];
TEBgMin = [-1.5; -1.5; -pi; -1.5];
TEBgMax = [ 1.5;  1.5;  pi; 1.5];
sD.grid = createGrid(TEBgMin, TEBgMax, TEBgN, 3);
data0 = sD.grid.xs{1}.^2 + sD.grid.xs{2}.^2;
extraArgs.obstacles = -data0; 

% Visualization
[TEBg2D, data02D] = proj(sD.grid,data0,[0 0 1 1],[0 0]);
figure(1)
clf
subplot(1,2,1)
surf(TEBg2D.xs{1}, TEBg2D.xs{2}, sqrt(data02D))
hold on

sD.dynSys = P4D_Q2D_Rel([]);
sD.uMode = 'min';
sD.dMode = 'max';
sD.accuracy = 'low';

f = figure(2);
set(f, 'Position', [400 400 450 400]);
extraArgs.visualize = true;
extraArgs.RS_level = 2; 
extraArgs.fig_num = 2;
extraArgs.plotData.plotDims = [1 1 0 0];
extraArgs.plotData.projpt = [0 0];
extraArgs.deleteLastPlot = false;

dt = 0.1;
tMax = 0.2;
tau = 0:dt:tMax;
extraArgs.stopConverge = true;
extraArgs.convergeThreshold = dt;
extraArgs.keepLast = true;

% solve backwards reachable set
% instead of none, do max_data0 - no obstacles or max over time
[data, tau] = HJIPDE_solve(data0, tau, sD, 'none', extraArgs); 

% largest cost along all time (along the entire 5th dimension which is
% time)
data = max(data,[],5); 

figure(1)
subplot(1,2,2)
[TEBg2D, data2D] = proj(sD.grid, data, [0 0 1 1], [0 0]);
s = surf(TEBg2D.xs{1}, TEBg2D.xs{2}, sqrt(data2D));
  
%Level set for tracking error bound
lev = min(min(s.ZData));


% More visualization
f = figure(3);
clf
set(f, 'Position', [360 278 560 420]);
alpha = .2;
small = .05;
  
levels = [.5, .75, 1];  
  
[g3D, data3D] = proj(sD.grid,data,[0 0 0 1], 0);
[~, data03D] = proj(sD.grid,data0,[0 0 0 1], 0);
  
  
for i = 1:3
    subplot(2,3,i)
    h0 = visSetIm(g3D, sqrt(data03D), 'blue', levels(i)+small);
    h0.FaceAlpha = alpha;
    hold on
    h = visSetIm(g3D, sqrt(data3D), 'red', levels(i));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small -pi pi])
  if i == 2
    title(['t = ' num2str(tau(end)) ' s'])
  end
    axis square
end
  
for i = 4:6
    subplot(2,3,i)
    h0 = visSetIm(TEBg2D, sqrt(data02D), 'blue', levels(i-3)+small);
    hold on
    h = visSetIm(TEBg2D, sqrt(data2D), 'red', levels(i-3));
    axis([-levels(3)-small levels(3)+small ...
      -levels(3)-small levels(3)+small])
    title(['R = ' num2str(levels(i-3))])
    axis square
          
set(gcf,'Color','white')
end


teb = sqrt(min(data(:))) + 0.03;
lev = lev + 0.03;
trackingRadius = lev;

figure(4)
clf
h0 = visSetIm(TEBg2D, sqrt(data02D), 'blue', lev+small);
hold on
h = visSetIm(TEBg2D, sqrt(data2D), 'red', lev);
title(['Tracking Error Bound R = ' num2str(lev)])
axis square

%% Planning
% Augment obstacles
augObs = addCRadius(g, obs, trackingRadius);
obs_bdry = -shapeRectangleByCorners(g, gMin + [0.1, 0.1], ...
       gMax - [0.1, 0.1]); 
augObs = min(augObs, obs_bdry);
targetLsmall = targetLength - trackingRadius;
targetsm = shapeRectangleByCorners(g, target - [targetLsmall; targetLsmall], ...
    target + [targetLsmall; targetLsmall]);

% Solve for BRS
schemeData.grid = g;
schemeData.uMode = 'min';
schemeData.dynSys = P;
extraArgs = [];
extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.plotData.plotDims = [1, 1];
extraArgs.plotData.projpt = [];
extraArgs.targets = targetsm;
extraArgs.obstacles = augObs;
extraArgs.stopInit = [47; 20];
BRS_tau = -55:0;
[BRS, tau] = HJIPDE_solve(targetsm, BRS_tau, schemeData, 'none', ...
  extraArgs);
BRS_tau = BRS_tau(end-length(tau)+1:end);

% Compute nominal trajectory
tauLength = length(BRS_tau);
iter = 1;
subSamples = 32;
dtSmall = (BRS_tau(2) - BRS_tau(1))/subSamples;
traj = nan(g.dim, tauLength);
traj(:,1) = P.x;
clns = repmat({':'}, 1, g.dim);
uMode = 'min';
projDim = [1 1];
showDims = find(projDim);
hideDims = ~projDim;
while iter <= tauLength
    BRS_at_t = BRS(clns{:}, tauLength-iter + 1);
    
    plot(traj(1, iter), traj(2, iter), 'ko')
    hold on
    [g2D, data2D] = proj(g, BRS_at_t, hideDims, traj(hideDims, iter));
    visSetIm(g2D, data2D);
    tStr = sprintf('t = %.3f\', BRS_tau(iter));
    title(tStr)
    drawnow
    hold off
    
    Deriv = computeGradients(g, BRS_at_t);
    for j = 1:subSamples
        deriv = eval_u(g, Deriv, P.x);
        u = P.optCtrl(BRS_tau(iter), P.x, deriv, uMode);
        P.updateState(u, dtSmall, P.x);
    end
    
    iter = iter + 1;
    traj(:,iter) = P.x;
end

%extraArgs = [];
%extraArgs.uMode = 'min';
%extraArgs.visualize = true;
%extraArgs.projDim = [1 1];
%extraArgs.subSamples = 32;
%[nomTraj, nomTraj_tau] = ...
%  computeOptTraj(g, BRS, BRS_tau, P, extraArgs);

%% Simulation
%Deriv = computeGradients(sD.grid, data);
%tStart = inf;
%tEnd = -inf;
%Q.x = Q.xhist(:,1);  
%Q.xhist = Q.x;     
%Q.u = [];            
%Q.uhist = [];         
%tStart = min(tStart, min(nomTraj_tau)); 
%tEnd = max(tEnd, max(nomTraj_tau));
%tau = tStart:obj.dt:tEnd;

%f = figure;

%h = visSetIm(g, augObs, 'k');
%h.LineWidth = 3;



end