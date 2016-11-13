function setRTTRS(obj, RTTRS)
% setRTTRS(obj, RTTRS)
%   Saves the RTTRS variable into the SPPProblem folder
%   Example:
%     load('RTTRS.mat')
%     SPPP.setRTTRS(RTTRS)

obj.RTTRS_filename = sprintf('%s/RTTRS.mat', obj.folder);
save(obj.RTTRS_filename, 'RTTRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end