function plotIntruderSim(obj, vehs, tPlot)
% plotIntruderSim(obj, tPlot)
%     Plots an intruder simulation

if nargin < 3
  tPlot = obj.tReplan;
end

% Load saved vehicle trajectories
if exist(obj.full_sim_filename, 'file')
  fprintf('Loading saved simulation results...\n')
  load(obj.full_sim_filename)
else
  error('Simulate results file not found!')
end

%% First plot: trajectories and overview
% Constants
f = figure;
f.Color = 'white';
f.Position = [100 100 530 360];

small = 1e-3;
arrowSize = 0.1;

% Initialize vehicles
Q = {Q1;Q2;Q3;Q4;Qintr};
colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

% Plot targets
hTar = plotTargetSets(Q(1:end-1), colors(1:4));

hAP = cell(length(Q), 1); % Actual position
hAT = cell(length(Q), 1); % Actual trajectory
hCR = cell(length(Q), 1); % Capture radius
hNP = cell(length(Q), 1); % Nominal position
for veh = 1:length(Q)
  tInd = find(Q{veh}.tau > tPlot - small & Q{veh}.tau < tPlot + small, 1);
  
  % Plot actual trajectory
  hAT{veh} = plot(Q{veh}.xhist(1,:), Q{veh}.xhist(2,:), ':', 'color', ...
    colors{veh});
  
  if ~isempty(tInd)
    % Plot vehicle states at tPlot
    x = Q{veh}.xhist(1,tInd);
    y = Q{veh}.xhist(2,tInd);
    t = Q{veh}.xhist(3,tInd);
    hAP{veh} = quiver(x, y, arrowSize*cos(t), arrowSize*sin(t), ...
      'color', colors{veh}, 'maxheadsize', 10, 'marker', '*');
    hCR{veh} = plotDisk([x; y], obj.Rc, '--', 'color', colors{veh});
    
    if veh < length(Q)
      % Plot original nominal trajectory
      hNP{veh} = plot(Q{veh}.nomTraj(1,tInd), Q{veh}.nomTraj(2,tInd), 'o', ...
        'color', colors{veh});
    end
  end
end

title(sprintf('t = %.2f', tPlot))
leg_handles = [hTar{2} hAP{2} hAP{3} hAP{4} hAP{5} hCR{2} hAT{2} hNP{2}];
leg_labels = {'Target', 'Vehicle 2', 'Vehicle 3', 'Vehicle 4', 'Intruder', ...
  'Danger zone', 'Trajectory', 'Nominal Position'};
legend(leg_handles, leg_labels, 'FontSize', 12, 'Location', 'eastoutside')
axis square

savefig(f, sprintf('%s_overview.fig', mfilename))

%% Second plot: Comparison between original and replanned trajectories
f = figure;
f.Color = 'white';
f.Position = [100 100 800 360];

hNTx = cell(length(Q),1); % Nominal trajectory x vs. t
hNTy = cell(length(Q),1); % Nominal trajectory y vs. t
hATx = cell(length(Q),1); % Actual trajectory x vs. t
hATy = cell(length(Q),1); % Actual trajectory y vs. t
hRT = cell(length(Q),1); % Replan time

% Plot original and planned trajectories
for i = 1:length(vehs)
  veh = vehs(i);
  subplot(2, 1, i)
  hNTx{veh} = plot(Q{veh}.nomTraj_tau, Q{veh}.nomTraj(1,:), 'k--');
  hold on
  hNTy{veh} = plot(Q{veh}.nomTraj_tau, Q{veh}.nomTraj(2,:), 'k-');
  
  hATx{veh} = plot(Q{veh}.tau(1:end), Q{veh}.xhist(1,:), '--', 'color', ...
    colors{veh});
  hATy{veh} = plot(Q{veh}.tau(1:end), Q{veh}.xhist(2,:), '-', 'color', ...
    colors{veh});  
  
  % Plot replan time
  hRT{veh} = plot([obj.tReplan obj.tReplan obj.tIntr obj.tIntr], ...
    [-2 2 2 -2], 'k:');
  
  % Legend
  leg_handles = [hNTx{veh} hNTy{veh} hATx{veh} hATy{veh} hRT{veh}];
  leg_labels = {'Nominal x traj.', 'Nominal y traj.','Actual x traj.', ...
    'Actual y traj.', 'Intruder time'};
  legend(leg_handles, leg_labels, 'FontSize', 12, 'Location', 'eastoutside')
  xlim([Q{veh}.tau(1) Q{veh}.tau(end)])
  ylim([-1 1])
  
  ylabel(sprintf('Q_%d Position', veh), 'FontSize', 12)
  
  if i == length(vehs)
    xlabel('Time', 'FontSize', 12)
  end
end
savefig(f, sprintf('%s_comparison.fig', mfilename))
end