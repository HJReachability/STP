function STP_FaSTrack_Sampada(data, sD, trackingRadius)
%% Setup
gMin = [0 0];
gMax = [50 50];
gN = [20 20];

g = createGrid(gMin, gMax, gN);
%temp_g = createGrid([-20 -20], [20 20], [15 15]);  
%obs = shapeRectangleByCorners(g, [30; 25], [35; 30]);
obs = shapeRectangleByCorners(g, [30; 20], [40; 30]);

vxmax = sD.dynSys.pxMax;
vymax = sD.dynSys.pyMax;
wmax = sD.dynSys.uMax;
aRange = [sD.dynSys.aMin, sD.dynSys.aMax];
dMax = sD.dynSys.dMax;


P = Plane2D([47; 20], vxmax, vymax);
dims = 1:4;
Q = Plane4D([47; 20; 0; 0], wmax, aRange, dMax, dims);

target = [20; 45];
targetLength = 6.5;
vehicleSize = 0.8;

%% Planning
% Augment obstacles
augObs = addCRadius(g, obs, trackingRadius + vehicleSize);
obs_bdry = -shapeRectangleByCorners(g, gMin + [0.1, 0.1], ...
       gMax - [0.1, 0.1]); 
augObs = min(augObs, obs_bdry);
targetLsmall = targetLength - trackingRadius - vehicleSize;
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

    %    [g2D, data2D] = proj(g, BRS_at_t, hideDims, traj(hideDims, iter));
    %visSetIm(g2D, data2D);
    visSetIm(g,BRS_at_t);
    tStr = sprintf('t = %.3f', BRS_tau(iter));
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
Deriv = computeGradients(sD.grid, data);

tStart = inf;
tEnd = -inf;
Q.x = Q.xhist(:,1);  
Q.xhist = Q.x;     
Q.u = [];            
Q.uhist = [];    
nomTraj_tau = BRS_tau(1:iter-1);
%tStart = min(tStart, min(nomTraj_tau)); 
%tEnd = max(tEnd, max(nomTraj_tau));
%tau = tStart:dtSmall:tEnd;

f = figure;
visSetIm(g, augObs, 'r');
hold on;
visSetIm(g, obs, 'k');
hold on;
rectangle('Position', [target(1)-targetLsmall target(2)-targetLsmall ...
    2*targetLsmall 2*targetLsmall]); 

for i = 1:length(nomTraj_tau)
    for s = 1:subSamples
        w = s/subSamples;
        prev_tInd = max(1, i-1);
        nomTraj_pt = (1-w)*traj(:,prev_tInd) + ...
          w*traj(:,i);
        rel_x = nomTraj_pt - Q.x(1:2); 
        rel_x(1:2) = rotate2D(rel_x(1:2), -Q.x(3));   
        rel_x(3:4) = [Q.x(3); Q.x(4)];

        deriv = eval_u(sD.grid, Deriv, rel_x);
        u = sD.dynSys.optCtrl([], rel_x, deriv, 'max');
        d = sD.dynSys.optDstb([], rel_x, deriv, 'min');
        d = [d(1) d(3)];
        Q.updateState(u, dtSmall/subSamples, Q.x, d);
    end
    Q.uhist(:, end-subSamples+1:end-1) = [];   
    Q.xhist(:, end-subSamples+1:end-1) = [];
    
    plot(Q.x(1), Q.x(2), 'b*');
    plot(traj(1, i), traj(2, i), 'ko');
    title(sprintf('t = %.0f', i));
    drawnow;
end
end
