function STP_FaSTrack_Sampada(data, sD, trackingRadius, DerivSim)
%% Setup
gMin = [0 0];
gMax = [50 50];
gN = [20 20];

g = createGrid(gMin, gMax, gN);
%temp_g = createGrid([-20 -20], [20 20], [15 15]);  
%obs = shapeRectangleByCorners(g, [30; 25], [35; 30]);
obs = shapeRectangleByCorners(g, [30; 20], [40; 30]);

%vxmax = sD.dynSys.pxMax;
%vymax = sD.dynSys.pyMax;
%wmax = sD.dynSys.uMax;
%aRange = [sD.dynSys.aMin, sD.dynSys.aMax];
%dMax = sD.dynSys.dMax;
wmax = 2*pi;
vxmax = 1;
vymax = 1;
dMax = [0.8, 0.8]; %%%%% making 2 disturbances
aRange = [-1.5, 1.5];

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
%DerivSim = computeGradients(sD.grid, data);

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

f = figure(13);
clf
visSetIm(g, augObs, 'r');
hold on;
visSetIm(g, obs, 'k');
hold on;
rectangle('Position', [target(1)-targetLsmall target(2)-targetLsmall ...
    2*targetLsmall 2*targetLsmall]); 

% for i = 1:length(nomTraj_tau)
%     nomTraj_pt = traj(:,i);
%     rel_x = nomTraj_pt - Q.x(1:2);
%     % maybe add rel_x(1:2) = rotate2D(rel_x(1:2), -Q.x(3));   
%     rel_x(3:4) = [Q.x(3); Q.x(4)];
%     deriv = eval_u(sD.grid, DerivSim, rel_x);
%     u = sD.dynSys.optCtrl([],rel_x,deriv,'min');
%     d = [0 0];
%     
%     Q.updateState(u, dtSmall, Q.x, d);
%     plot(Q.x(1), Q.x(2), 'b*');
%     plot(traj(1, i), traj(2, i), 'ko');
%     title(sprintf('t = %.0f', i));
%     drawnow;
% end

% for i = 1:length(nomTraj_tau)
%     for s = 1:subSamples
%         w = s/subSamples;
%         prev_tInd = max(1, i-1);
%         nomTraj_pt = traj(:,i);
%         nomTraj_pt = (1-w)*traj(:,prev_tInd) + ...
%          w*traj(:,i);
%         rel_x = nomTraj_pt - Q.x(1:2); 
%         rel_x(1:2) = rotate2D(rel_x(1:2), -Q.x(3));   
%         rel_x(3:4) = [Q.x(3); Q.x(4)];
% 
%         deriv = eval_u(sD.grid, DerivSim, rel_x);
%         u = sD.dynSys.optCtrl([], rel_x, deriv, 'min');
%         d = sD.dynSys.optDstb([], rel_x, deriv, 'max');
%         d = [d(1) d(3)];
%         Q.updateState(u, dtSmall/subSamples, Q.x, d);
%     end
%     Q.uhist(:, end-subSamples+1:end-1) = [];   
%     Q.xhist(:, end-subSamples+1:end-1) = [];
%     
%     plot(Q.x(1), Q.x(2), 'b*');
%     plot(traj(1, i), traj(2, i), 'ko');
%     title(sprintf('t = %.0f', i));
%     drawnow;
% end


%% Start loop! Tracking error bound block
%input: environment, sensing
%output: augmented obstacles
%newStates = [];
vis = 1;
iter = 0;
%global_start = tic; % Time entire simulation

max_iter = length(nomTraj_tau);%5000;
lookup_time = 0;
dt = 1;
uMode = 'max';

while iter < max_iter && norm(Q.x([1 2]) - target) > 0.5
  iter = iter + 1;

  % 1. Sense your environment, locate obstacles
  % 2. Expand sensed obstacles by tracking error bound
  %sensed_new = obsMap.sense_update(trueQuad.x([1 5 9]), senseRange, trackErr);
  
  %% Path Planner Block
  % Replan if a new obstacle is seen
  %if isempty(newStates) || sensed_new
    % Update next virtual state
  %  newStates = rrtNextState(trueQuad.x([1 5 9]), goal, obsMap.padded_obs, ...
  %    delta_x, [], false);
  %end
  %virt_x = newStates(1,:)';
  %newStates(1,:) = [];
  virt_x = traj(:,iter);

  %% Hybrid Tracking Controller
  % 1. find relative state
  local_start = tic;  
  rel_x = virt_x - Q.x(1:2); 
rel_x(1:2) = rotate2D(rel_x(1:2), Q.x(3));   
rel_x(3:4) = [Q.x(3); Q.x(4)];
  
  % 2. Determine which controller to use, find optimal control
  %get spatial gradients
  p = eval_u(sD.grid, DerivSim, rel_x);
  
  % Find optimal control of relative system (no performance control)
  u = sD.dynSys.optCtrl([], rel_x, p, uMode);
  lookup_time = lookup_time + toc(local_start);
  
  %% True System Block
  % 1. add random disturbance to velocity within given bound
  d = sD.dynSys.dMin + rand(2,1).*(sD.dynSys.dMax - sD.dynSys.dMin);
  
  % 2. update state of true vehicle
  Q.updateState(u, dt, [], d);

  % Make sure error isn't too big (shouldn't happen)
  if norm(virt_x - Q.x([1 2])) > 3
    keyboard
  end
  
  %% Virtual System Block  
%   fprintf('Iteration took %.2f seconds\n', toc);
  if vis
    
    plot(Q.x(1), Q.x(2), 'b*');
    plot(traj(1, iter), traj(2, iter), 'ko');
    title(sprintf('t = %.0f', iter));
    drawnow;
%     export_fig(sprintf('pics/%d', iter), '-png')
%     savefig(sprintf('pics/%d.fig', iter))    
  end
end
end
