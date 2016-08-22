function resimulate(obj)

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
if exist(obj.resim_filename, 'file')
  fprintf('Saved simulation results...\n')
  load(obj.resim_filename)
else
  error('Simulate reults file not found!')
end

Q = {Q1;Q2;Q3;Q4;Qintr};
colors = lines(4);
small = 1e-4;

firstPlot = true(length(Q), 1);
hPosition = cell(length(Q), 1);
hCapRadius = cell(length(Q), 1);
hObstacles = cell(length(Q), 1);

for i = 1:length(obj.tau)
  for veh = 1:length(Q)
    tInd = find(Q{veh}.data.tau > obj.tau(i) - small & ...
      Q{veh}.data.tau < obj.tau(i) + small, 1);
    
    if ~isempty(tInd)
      xTraj = Q{veh}.xhist(1,1:tInd+1);
      yTraj = Q{veh}.xhist(2,1:tInd+1);
      if firstPlot(veh)
        hPosition{veh} = plot(xTraj, yTraj, '.', 'color', colors(veh,:));
        hCapRadius{veh} = plotDisk([xTraj(end); yTraj(end)], CARS.Rc, ...
          'color', colors(veh,:));
        
      else
        hPosition{veh}.XData = xTraj;
        hPosition{veh}.YData = yTraj;
        
        [~, diskX, diskY] = plotDisk([xTraj(end); yTraj(end)], CARS.Rc);
        hCapRadius{veh}.XData = diskX;
        hCapRadius{veh}.YData = diskY;
      end
    end
  end
end

end