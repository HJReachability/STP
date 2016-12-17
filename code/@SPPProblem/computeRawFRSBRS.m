function computeRawFRSBRS(obj, save_png)
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

g = createGrid([-50; -50; -pi], [50; 50; pi], [101; 101; 21], 3);
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

FRSBRS.BRS = cell(length(CARS.tau), 1);
FRSBRS.BRS{1} = FRSBRS.FRS.data(:,:,:,1);

for i = 2:length(CARS.tau)
  data0 = FRSBRS.FRS.data(:,:,:,i);
  tau = CARS.tau(end-i+1:end);
  
  extraArgs.fig_filename = sprintf('%s/BRS%d_', folder, i);
  FRSBRS.BRS{i}.data = HJIPDE_solve(data0, tau, sD_BRS, 'zero', extraArgs);
  FRSBRS.BRS{i}.tau = tau;
end

% Update SPPP and save
obj.FRSBRS_filename = sprintf('%s/FRSBRS.mat', obj.folder);
save(obj.FRSBRS_filename, 'FRSBRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end