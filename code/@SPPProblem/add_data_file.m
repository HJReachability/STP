function add_data_file(obj, data_name, data_file)
% add_data_file(obj, data_name, data_file)
%   Add a data file to the SPPProblem object
%   Example:
%     SPPP = SPPProblem('SF_dstb_11_4sSep')
%     SPPP.add_data_file('RTTRS', 'RTTRS6.mat')

%% Copy file to object folder
field_name = sprintf('%s_filename', data_name);
obj.(field_name) = sprintf('%s/%s.mat', obj.folder, data_name);

if ispc
  alt_dest_filename = sprintf('%s\\%s.mat', obj.folder, data_name);
  cmd = sprintf('copy %s %s', data_file, alt_dest_filename);
  fprintf('Executing %s.\n', cmd)
  system(cmd);
else
  system(sprintf('cp %s %s', data_file, obj.(field_name)));
end

%% Special case for CARS property
if strcmp(data_name, 'CARS')
  fprintf('Setting CARS-related properties.\n')
  load(obj.CARS_filename)

  obj.buffer_duration = max(CARS.tau)/obj.max_num_affected_vehicles;
  obj.buffer_duration_ind = find(CARS.tau >= obj.buffer_duration, 1, 'first');
  obj.remaining_duration_ind = length(CARS.tau) - obj.buffer_duration_ind;
end

%% Save object
SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end