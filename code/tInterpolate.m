function dataOut = tInterpolate(tau, dataIn, t)
% dataOut = tInterpolate(tau, dataIn, t)
%
% Given data dataIn and corresponding time points tau, compute the
% output data dataOut at times t using linear interpolation in time.
%
% dataIn and dataOut's rows correspond to different times
%
% Mo Chen, 2014-10-13
%

% Data sizes
m = size(dataIn,2);
dataOut = zeros(length(t), m);

% Minimum and maximum time
[tauMin, imin] = min(tau);
[tauMax, imax] = max(tau);

% Indices of times before and after minimum and maximum times
im = logical(t<tauMin);
ip = logical(t>tauMax);
ii = logical(t>=tauMin & t<=tauMax);

% Interpolate
for i = 1:m
    dataOut(im, i) = dataIn(imin, i);
    dataOut(ip, i) = dataIn(imax, i);
    dataOut(ii, i) = interpn(tau, dataIn(:, i), t(ii));
end

end