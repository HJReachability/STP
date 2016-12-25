function setCARS(obj, CARS)
% setRTTRS(obj, RTTRS)
%   Saves the RTTRS variable into the SPPProblem folder
%   Example:
%     load('RTTRS.mat')
%     SPPP.setRTTRS(RTTRS)

obj.CARS_filename = sprintf('%s/CARS.mat', obj.folder);
save(obj.CARS_filename, 'CARS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end