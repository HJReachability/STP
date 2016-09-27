function plotIntruderSim(obj, tPlot)
% plotIntruderSim(obj, tPlot)
%     Plots an intruder simulation

if nargin < 2
  tPlot = -2.38;
end

% Load saved vehicle trajectories
if exist(obj.full_sim_filename, 'file')
  fprintf('Loading saved simulation results...\n')
  load(obj.full_sim_filename)
else
  error('Simulate results file not found!')
end

% Constants
f = figure;
f.Color = 'white';
f.Position = [100 100 800 800];

small = 1e-3;
arrowSize = 0.1;

% Initialize vehicles
Q = {Q1;Q2;Q3;Q4;Qintr};
colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

% Plot targets
plotTargetSets(Q(1:end-1), colors(1:4));

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
      hNP{veh} = plot(Q{veh}.nomTraj(1,tInd), Q{veh}.nomTraj(2,tInd), '--', ...
        'color', colors{veh});
    end
  end
  
  
end
end