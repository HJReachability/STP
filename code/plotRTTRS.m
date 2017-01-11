thetaMin = -1;
thetaMax = 1;
N = 15;

thetas = linspace(thetaMin, thetaMax, N);

figure
spC = ceil(sqrt(N)) + 1;
spR = ceil(N/spC);

for i = 1:N
  subplot(spR, spC, i)
  
  [g2D, data2D] = proj(RTTRS.g, RTTRS.data, [0 0 1], thetas(i));
%   visSetIm(g2D, data2D, [], []);
  surf(g2D.vs{1}, g2D.vs{2}, data2D', 'linestyle', 'none')
  title(sprintf('\\theta = %.2f', thetas(i)))
end

xMin = -0.5;
xMax = 0.5;
xs = linspace(xMin, xMax, N);

figure
spC = ceil(sqrt(N)) + 1;
spR = ceil(N/spC);

for i = 1:N
  subplot(spR, spC, i)
  
  [g2D, data2D] = proj(RTTRS.g, RTTRS.data, [1 0 0], xs(i));
%   visSetIm(g2D, data2D, [], []);
  surf(g2D.vs{1}, g2D.vs{2}, data2D', 'linestyle', 'none')
  title(sprintf('x = %.2f', xs(i)))
end