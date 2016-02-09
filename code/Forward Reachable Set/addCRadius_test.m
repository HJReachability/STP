function addCRadius_test()

%% Create the computation grid.
g.dim = 2;
g.min = [-10; -10];
g.max = [+10; +10];
g.N = [101; 101];
g.bdry = @addGhostExtrapolate;
g = processGrid(g);

%% Run N random instances
N = 9;

figure;
spC = ceil(sqrt(N));
spR = ceil(N/spC);
for i = 1:N
  subplot(spR, spC, i)
  addCRadius_single(g)
  drawnow
end
end

function addCRadius_single(g)
%% Define the intial set
% Sphere
sc = -1 + 2*rand(2,1);
r = 0.5 + 3.5*rand;
data1 = shapeSphere(g, sc, r);

% Rectangle
rc = 1 + 2*rand(2,1);
w = 0.5 + 3.5*rand;
data2 = shapeRectangleByCenter(g, rc, w);

% Union
data = min(data1, data2);

%% Plot intial set
contour(g.xs{1}, g.xs{2}, data, [0 0], 'b')
hold on

%% Add random radius and plot resulting set
extra_radius = 1 + 4*rand;
dataOut = addCRadius(g, data, extra_radius);

contour(g.xs{1}, g.xs{2}, dataOut, [0 0], 'r')
title(['Extra radius: ' num2str(extra_radius)])
end