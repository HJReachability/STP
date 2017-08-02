function SPPwFaSTrack(problem_name, extraArgs)
% SPPwFaSTrack()
%     Solves the entire SPP with disturbances problem using the RTT method
%     and FaSTrack

if nargin < 2
  extraArgs = [];
end

if isfield(extraArgs, 'SPPP')
  SPPP = extraArgs.SPPP;
else
  SPPP = SPPProblemFaSTrack(problem_name, extraArgs);
end


SPPP.computeTEB();
SPPP.computeNIRS();
SPPP.simulateNI();
end