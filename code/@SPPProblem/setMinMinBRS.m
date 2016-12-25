function setMinMinBRS(obj, minMinBRS)
% setRTTRS(obj, RTTRS)
%   Saves the RTTRS variable into the SPPProblem folder
%   Example:
%     load('RTTRS.mat')
%     SPPP.setRTTRS(RTTRS)

obj.minMinBRS_filename = sprintf('%s/minMinBRS.mat', obj.folder);
save(obj.minMinBRS_filename, 'minMinBRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end