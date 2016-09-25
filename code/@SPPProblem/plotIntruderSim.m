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

%% Constants
f = figure;
f.Color = 'white';
f.Position = [100 100 800 800];

small = 1e-4;
arrowSize = 0.1;

%% Initialize vehicles
Q = {Q1;Q2;Q3;Q4;Qintr};
colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

%% Plot targets
plotTargetSets(Q(1:end-1), colors(1:4));

end