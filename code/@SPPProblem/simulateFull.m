function simulateFull(obj, save_png, save_fig)
% simulateFull(obj, save_png, save_fig)
%     Reruns the simulation using saved trajectory
if nargin < 2
  save_png = true;
end

if nargin < 3
  save_fig = false;
end

%% Load files
% Load robust tracking reachable set
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

% Load safety reachable set
if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

% Load saved vehicle trajectories
if exist(obj.full_sim_filename, 'file')
  fprintf('Loading saved simulation results...\n')
  load(obj.full_sim_filename)
else
  error('Simulate results file not found!')
end

% Load raw obstacles file
if exist(obj.rawAugObs_filename, 'file')
  fprintf('Loading ''raw'' obstacles...\n')
  load(obj.rawAugObs_filename)
else
  error('Raw obstacles file not found!')
end

Q = {Q1;Q2;Q3;Q4;Qintr};
colors = {'b', 'r', [0 0.5 0], 'm', 'k'};
small = 1e-4;

firstPlot = true(length(Q), 1);
hPosition = cell(length(Q), 1);
hHeading = cell(length(Q), 1);
hCapRadius = cell(length(Q), 1);
hObstacles = cell(length(Q), 1);
hNomTraj = cell(length(Q)-1, 1);

% Add cylindrical obstacles for visualization
if save_png || save_fig
  folder = sprintf('%s_%f', mfilename, now);
  system(sprintf('mkdir %s', folder));
end

f = figure;
arrowSize = 0.1;
plotTargetSets(Q(1:end-1), colors(1:4));
f.Position = [100 100 800 800];
axis square
hold on

for veh = 1:length(Q)-1
  fprintf('Adding obstacles for vehicle %d for visualization...\n', veh)
  Q{veh}.addObs2D(obj, RTTRS, CARS, rawAugObs);
end

for i = 1:length(obj.tau)
  for veh = 1:length(Q)
    ti = find(Q{veh}.tau > obj.tau(i)-small & Q{veh}.tau < obj.tau(i)+small, 1);
    
    if ~isempty(ti)
      xTraj = Q{veh}.xhist(1,1:ti);
      yTraj = Q{veh}.xhist(2,1:ti);
      x = xTraj(end);
      y = yTraj(end);
      
      t = Q{veh}.xhist(3,ti);
      
      if firstPlot(veh)
        % Position, capture radius, heading
        hPosition{veh} = plot(xTraj, yTraj, '.', 'color', colors{veh}, ...
          'MarkerSize', 1);
        hCapRadius{veh} = plotDisk([x; y], obj.Rc, 'color', colors{veh}, ...
          'linestyle', '--', 'linewidth', 2);
        hHeading{veh} = quiver(x, y, arrowSize*cos(t), arrowSize*sin(t), ...
          'color', colors{veh}, 'maxheadsize', 10, 'marker', '*');
        
        if veh < length(Q)
          % SPP Vehicle obstacles
          obs2D = Q{veh}.obs2D(:,:,ti);
          hObstacles{veh} = visSetIm(obj.g2D, obs2D, colors{veh});
          hObstacles{veh}.LineStyle = ':';
          hObstacles{veh}.LineWidth = 2;
          
          xNomTraj = Q{veh}.nomTraj(1,ti);
          yNomTraj = Q{veh}.nomTraj(2,ti);
          hNomTraj{veh} = plot(xNomTraj, yNomTraj, 'o', 'color', colors{veh});
        else
          % Intruder extra plots
          hHeading{veh}.Marker = '.';
          hHeading{veh}.MarkerSize = 25;
        end
        
        firstPlot(veh) = false;
      else
        hPosition{veh}.XData = xTraj;
        hPosition{veh}.YData = yTraj;
        
        [~, diskX, diskY] = plotDisk([x; y], obj.Rc);
        hCapRadius{veh}.XData = diskX;
        hCapRadius{veh}.YData = diskY;
        
        hHeading{veh}.XData = x;
        hHeading{veh}.YData = y;
        hHeading{veh}.UData = arrowSize*cos(t);
        hHeading{veh}.VData = arrowSize*sin(t);
        
        if veh < length(Q)
          obs2D = Q{veh}.obs2D(:,:,ti);
          hObstacles{veh}.ZData = obs2D;
          
          xNomTraj = Q{veh}.nomTraj(1,ti);
          yNomTraj = Q{veh}.nomTraj(2,ti);          
          hNomTraj{veh}.XData = xNomTraj;
          hNomTraj{veh}.YData = yNomTraj;
        else

        end
      end
    end
  end
  title(sprintf('t = %.2f', obj.tau(i)))
  drawnow
  
  % Save plots
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
end
end