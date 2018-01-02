function SPPwFaSTrack(problem_name, extraArgs)
% SPPwFaSTrack()
%     Solves the entire SPP with disturbances problem using FaSTrack

if nargin < 2
  extraArgs = [];
end

if isfield(extraArgs, 'SPPP')
  SPPP = extraArgs.SPPP;
else
  SPPP = SPPProblemFaSTrack(problem_name, extraArgs);
end


SPPP.computeTEB();
SPPP.computeNIRSFaSTrack();
SPPP.simulateNIFaSTrack();
end