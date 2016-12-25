function setFRSBRS(obj, FRSBRS)
% setFRSBRS(obj, FRSBRS)
%   Saves the FRSBRS variable into the SPPProblem folder
%   Example:
%     load('FRSBRS.mat')
%     SPPP.setFRSBRS(FRSBRS)

obj.FRSBRS_filename = sprintf('%s/FRSBRS.mat', obj.folder);
save(obj.FRSBRS_filename, 'FRSBRS', '-v7.3')

SPPP = obj;
save(sprintf('%s/SPPP.mat', obj.folder), 'SPPP', '-v7.3')
end