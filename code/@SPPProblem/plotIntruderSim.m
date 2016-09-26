function plotIntruderSim(obj, tPlot)
% plotIntruderSim(obj, tPlot)
%     Plots an intruder simulation

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

small = 1e-4;
arrowSize = 0.1;

% Initialize vehicles
Q = {Q1;Q2;Q3;Q4;Qintr};
colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

% Plot targets
plotTargetSets(Q(1:end-1), colors(1:4));

hAT = cell(length(Q), 1);
hOT = cell(length(Q), 1);
for veh = 1:length(Q)
  % Plot actual trajectory
  hAT{veh} = plot(Q{veh}.xhist(1,:), Q{veh}.xhist(1,:), ':', color, ...
    colors{veh});
  
  % Plot vehicle states at tPlot
  hAT
  
  if veh < length(Q)
    % Plot original nominal trajectory
    hOT{veh} = plot(Q{veh}.nomTraj(1,:), Q{veh}.nomTraj(2,:), '--', color, ...
      colors{veh});
  end
end
end