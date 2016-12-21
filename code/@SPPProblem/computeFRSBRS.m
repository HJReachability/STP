function computeFRSBRS(obj, save_png)
% computeRawFRSBRS(obj, save_png)

if nargin < 2
  save_png = true;
end

fprintf('Executing %s...\n', mfilename)

if save_png
  if ispc
    folder = sprintf('%s\\%s', obj.folder, mfilename);
  else
    folder = sprintf('%s/%s', obj.folder, mfilename);
  end
  system(sprintf('mkdir %s', folder));
end

%% Load files
% RTTRS
if exist(obj.RTTRS_filename, 'file')
  fprintf('Loading RTTRS...\n')
  load(obj.RTTRS_filename)
else
  error('RTTRS file not found!')
end

% CARS
if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

% g = createGrid([-75; -75; -pi], [75; 75; pi], [101; 101; 21], 3); % 11 m/s wd
g = createGrid([-71; -71; -pi], [71; 71; pi], [251; 251; 21], 3); % 6 m/s wd
RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, g);

% Computes FRS of RTTRS
dynSys = Plane(zeros(3,1), obj.wMaxA, obj.vRangeA, obj.dMaxA);

sD_FRS.grid = g;
sD_FRS.uMode = 'max';
sD_FRS.dMode = 'max';
sD_FRS.tMode = 'forward';
sD_FRS.dynSys = dynSys;

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;
extraArgs.fig_filename = sprintf('%s/FRS', folder);
  
FRSBRS.FRS.data = HJIPDE_solve(RTTRSdata, CARS.tau, sD_FRS, 'none', extraArgs);
FRSBRS.FRS.tau = CARS.tau;
FRSBRS.g = g;

% Compute BRS(t) of FRS(t) for each t
sD_BRS.grid = g;
sD_BRS.uMode = 'min';
sD_BRS.dMode = 'min';
sD_BRS.tMode = 'backward';
sD_BRS.dynSys = dynSys;

FRSBRS.BRS.data = cell(length(CARS.tau), 1);
FRSBRS.BRS.tau = cell(length(CARS.tau), 1);

for i = length(CARS.tau):-1:1
  [~, FRS2D] = proj(g, FRSBRS.FRS.data(:,:,:,i), [0 0 1]);
  flat_FRS = repmat(FRS2D, [1 1 g.N(3)]);
  
  extraArgs.fig_filename = sprintf('%s/BRS%d_', folder, i);
  FRSBRS.BRS.data{i} = HJIPDE_solve(flat_FRS, CARS.tau, sD_BRS, 'zero', ...
    extraArgs);    

end

% Update SPPP and save
obj.FRSBRS_filename = sprintf('%s/FRSBRS.mat', obj.folder);
save(obj.FRSBRS_filename, 'FRSBRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end