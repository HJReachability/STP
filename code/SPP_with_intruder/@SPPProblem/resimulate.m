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
small = 1e-4;
for i = 1:length(obj.tau)
  for veh = 1:length(Q)
    tInd = find(Q{veh}.data.nomTraj_tau > tauBR(i) - small & ...
      Q{veh}.data.nomTraj_tau < tauBR(i) + small, 1);
  end
end

end