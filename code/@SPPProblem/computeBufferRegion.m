function computeBufferRegion(obj, save_png)
% bufferRegion = computeBufferRegion(RTTRSdata, CARS, SPPP, g, kBar, save_png)

if nargin < 2
  save_png = true;
end

fprintf('Executing %s...\n', mfilename)

% if exist(obj.bufferRegion_filename, 'file')
%   fprintf(['The buffer region file %s already exists. Skipping ' ...
%     'computation.\n'], ...
%     obj.bufferRegion_filename)
%   return
% end

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

% minMinBRS
if exist(obj.minMinBRS_filename, 'file')
  fprintf('Loading minMinBRS...\n')
  load(obj.minMinBRS_filename)
else
  error('minMinBRS file not found!')
end

% Buffer region duration and remaining duration indices
kBar = obj.max_num_affected_vehicles;
tBRD = max(CARS.tau)/kBar;
tBRD_ind = find(CARS.tau >= tBRD, 1, 'first');
tRD_ind = length(CARS.tau) - tBRD_ind;

% Migrate to common grid
% g = createGrid([-80; -80; -pi], [80; 80; pi], [55; 55; 21], 3); % 11 m/s wind
g = createGrid([-55; -55; -pi], [55; 55; pi], [75; 75; 21], 3); % 6 m/s wind
RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, g);
CARS_final = migrateGrid(CARS.g, CARS.data(:,:,:,end), g);
CARS_RD = migrateGrid(CARS.g, CARS.data(:,:,:,tRD_ind), g);
minMinBRS_BRD = migrateGrid(minMinBRS.g, minMinBRS.data(:,:,:,tBRD_ind), g);

pdims = [1 2];
adim = 3;
bdry_only = false;

%% Intruder affects this vehicle first
% Augment base obstacle with max min set to get barA1
barA1A = computeDataByUnion(g, CARS_final, g, RTTRSdata, pdims, adim, ...
  bdry_only);

if save_png
  figure
  visSetIm(g, barA1A, 'r');
  export_fig(sprintf('%s/barA1A', folder), '-png')
end

% Augment barA1 with min min set to get barA2
barA2A = computeDataByUnion(g, minMinBRS_BRD, g, barA1A, pdims, adim, ...
  bdry_only);

if save_png
  figure
  visSetIm(g, barA2A, 'b');
  export_fig(sprintf('%s/barA2A', folder), '-png')
end

% Augment barA2 with max min set of a shorter duration
bufferRegionA = computeDataByUnion(g, CARS_RD, g, barA2A, pdims, adim, ...
  bdry_only);

if save_png
  figure
  visSetIm(g, bufferRegionA, [0 0.75 0]);
  export_fig(sprintf('%s/bufferRegionA', folder), '-png')
end

%% Intruder affects other vehicle first
% Augment base obstacle with max min set to get barA1
barA1B = computeDataByUnion(g, CARS_RD, g, RTTRSdata, pdims, adim, bdry_only);

if save_png
  figure
  visSetIm(g, barA1B, 'r');
  export_fig(sprintf('%s/barA1B', folder), '-png')
end

% Augment barA1 with min min set to get barA2
barA2B = computeDataByUnion(g, minMinBRS_BRD, g, barA1B, pdims, adim, ...
  bdry_only);

if save_png
  figure
  visSetIm(g, barA2B, 'b');
  export_fig(sprintf('%s/barA2B', folder), '-png')
end

% Augment barA2 with max min set of a shorter duration
bufferRegionB = computeDataByUnion(g, CARS_final, g, barA2B, pdims, adim, ...
  bdry_only);

if save_png
  figure
  visSetIm(g, bufferRegionB, [0 0.75 0]);
  export_fig(sprintf('%s/bufferRegionB', folder), '-png')
end

% Overall buffer region
bufferRegion.data = min(bufferRegionA, bufferRegionB);
bufferRegion.g = g;

if save_png
  figure
  visSetIm(g, bufferRegion.data);
  export_fig(sprintf('%s/bufferRegion', folder), '-png')
end

% Update SPPP and save
obj.bufferRegion_filename = sprintf('%s/bufferRegion.mat', obj.folder);
save(obj.bufferRegion_filename, 'bufferRegion', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')

end