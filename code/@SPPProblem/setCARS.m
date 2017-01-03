function setCARS(obj, CARS)
% setRTTRS(obj, RTTRS)
%   Saves the RTTRS variable into the SPPProblem folder
%   Example:
%     load('CARS.mat')
%     SPPP.setCARS(CARS)

obj.CARS_filename = sprintf('%s/CARS.mat', obj.folder);
save(obj.CARS_filename, 'CARS', '-v7.3')

obj.buffer_duration = max(CARS.tau)/obj.max_num_affected_vehicles;
obj.buffer_duration_ind = find(CARS.tau >= obj.buffer_duration, 1, 'first');
obj.remaining_duration_ind = length(CARS.tau) - obj.buffer_duration_ind;

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end