function setBufferRegion(obj, bufferRegion)
% setRTTRS(obj, RTTRS)
%   Saves the RTTRS variable into the SPPProblem folder
%   Example:
%     load('RTTRS.mat')
%     SPPP.setRTTRS(RTTRS)

obj.bufferRegion_filename = sprintf('%s/bufferRegion.mat', obj.folder);
save(obj.bufferRegion_filename, 'bufferRegion', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end