function [t, i] = latestRS(x, RS)
% [t, i] = latestRS(x, RS)
%
% Computes the time and index of the latest reachable set that includes the
% state x
%
% Inputs:  x  - state of interest
%          RS - reachable set structure
%
% Outputs: t - time stamp
%          i - index
%
% Mo Chen, 2016-02-06

if numel(x) ~= RS.g.dim
  error('Dimensions of input state and grid don''t match!')
end

for i = 1:length(RS.tau)
  t = RS.tau(i);
  
  if eval_u(RS.g, RS.data(:,:,:,i), x) <= 0
    return
  end
end

warning('x is still outside of the reachable set')

end