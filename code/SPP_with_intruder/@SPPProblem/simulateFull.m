function simulateFull(obj)

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
colors = lines(4);
small = 1e-4;

firstPlot = true(length(Q), 1);
hPosition = cell(length(Q), 1);
hHeading = cell(length(Q), 1);
hCapRadius = cell(length(Q), 1);
hObstacles = cell(length(Q), 1);

figure
arrowSize = 0.2;
plotTargetSets(Q(1:end-1), colors(1:4, :));
hold on

for veh = 1:length(Q)-1
  fprintf('Adding obstacles for vehicle %d for visualization...\n', veh)
  Q{veh}.addObs2D(obj, RTTRS, CARS, rawAugObs);
end

for i = 1:length(obj.tau)
  for veh = 1:length(Q)
    tInd = ...
      find(Q{veh}.tau > obj.tau(i)-small &  Q{veh}.tau < obj.tau(i)+small, 1);
    
    if ~isempty(tInd)
      xTraj = Q{veh}.xhist(1,1:tInd+1);
      yTraj = Q{veh}.xhist(2,1:tInd+1);
      x = xTraj(end);
      y = yTraj(end);
      
      t = Q{veh}.getHeading();
      
      obs2D = Q{veh}.obs2D(:,:,tInd);
      
      if firstPlot(veh)
        % Position, capture radius, heading
        hPosition{veh} = plot(xTraj, yTraj, '.', 'color', colors(veh,:));
        hCapRadius{veh} = plotDisk([x; y], obj.Rc, 'color', colors(veh,:));
        hHeading{veh} = quiver(x, y, arrowSize*cos(t), arrowSize*sin(t), ...
          'color', colors(veh,:));
        
        if veh < length(Q)
          % SPP Vehicle extra plots
          hObstacles{veh} = visSetIm(obj.g2D, obs2D, colors(veh,:));
        else
          % Intruder extra plots
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
        hHeading{veh}.UData = cos(t);
        hHeading{veh}.VData = sin(t);
        
        if veh < length(Q)
          hObstacles{veh}.ZData = obs2D;
        else
        end
      end
    end
  end
  title(sprintf('t = %.2f', obj.tau(i)))
  drawnow
end
end