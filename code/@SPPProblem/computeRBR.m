function computeRBR(obj, all_tau)

if nargin < 2
  all_tau = false;
end


load(obj.CARS_filename)
load(obj.minMinBRS_filename)

% Put everything in minMinBRS's grid
rdi = obj.remaining_duration_ind;
CARSdata = migrateGrid(CARS.g, CARS.data(:,:,:,rdi), minMinBRS.g);

bdi = obj.buffer_duration_ind;

if all_tau
  schemeData.grid = minMinBRS.g;
  schemeData.dynSys = minMinBRS.dynSys;
  extraArgs.visualize = true;
  RBR.data = HJIPDE_solve(CARSdata, CARS.tau(1:bdi), schemeData, 'zero', ...
    extraArgs);
else
  
  RBR.data = computeDataByUnion(minMinBRS.g, minMinBRS.data(:,:,:,bdi), ...
    minMinBRS.g, CARSdata, [1 2], 3, false);

end

RBR.g = minMinBRS.g;

obj.RBR_filename = sprintf('%s/RBR.mat', obj.folder);
save(obj.RBR_filename, 'RBR', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
  
end