function tracking_test()

addpath(genpath('.'))

%% Problem parameters
tradius = 0.1;
speed = 0.75;
uMax = 0.6;
dMax = [0; 0];

%% Compute or reachable set
filename = ['BRS_' num2str(tradius) '_' num2str(speed) '_' num2str(uMax) ...
  '_' num2str(dMax(1)) num2str(dMax(2)) '.mat'];
if exist(filename, 'file')
  load(filename)
else
  BRS = computeBRS(tradius, speed, uMax, dMax);
  save(filename, 'BRS')
end

%% Create trajectory
IS = [-0.7; -0.3; 0];
plv = Plane(IS, speed);

% Plot initial setup
figure;
plv.plotPosition(); % Initial position
hold on

% Target set
theta = linspace(0, 2*pi, 100);
plot(tradius*cos(theta), tradius*sin(theta), 'r-')

xlim([-1 1])
ylim([-1 1])
title('t = 0')
drawnow;

dt = 0.025;
tMax = 10;
t = 0:dt:tMax;

for i = 1:length(t)
  x = plv.x;
  x(3) = wrapTo2Pi(x(3));
  
  if norm(plv.getPosition()) <= tradius
    break
  end
  
  p = calculateCostate(BRS.g, BRS.P, x);
  
  u = (p(3) >= 0) * (-uMax) + (p(3) < 0) * uMax;
  plv.updateState(u, dt);
  plv.plotPosition();
  title(['t = ' num2str(t(i))])
  drawnow;
end

%% Load bubble
bradius = 0.1;
vNom = 0.75;
vRange = [0.5; 1];
wNom = 0.6;
wMax = 1;
dMax = [0.1; 0.2];

filename = ['RB_' num2str(bradius) '_' num2str(vNom) '_' num2str(vRange(1)) ...
  num2str(vRange(1)) '_' num2str(wNom) '_' num2str(wMax) '_' ...
  num2str(dMax(1)) num2str(dMax(2)) '.mat'];

if exist(filename, 'file')
  load(filename)
else
  RB = computeRB(bradius, vNom, vRange, wNom, wMax, dMax);
  save(filename, 'RB')
end

%% Follow trajectory
pl = Plane(IS);

% Plot initial setup
figure;
pl.plotPosition(); % Initial position
hold on
theta = linspace(0, 2*pi, 100);
plot(tradius*cos(theta), tradius*sin(theta), 'r-')

% Target set
[g2D, data2D] = proj2D(BRS.g, BRS.data(:,:,:,1), [0 0 1]);
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'r')

xlim([-1 1])
ylim([-1 1])
title('t = 0')
drawnow;

for i = 1:size(plv.xhist, 2)
  % Bubble control
  u = RBControl(pl.x, plv.xhist(:, i), vRange, [-wMax wMax], RB);
  
  not_done = true;
  while not_done
    d12 = -dMax(1) + 2*dMax(1)*rand(2, 1);
    not_done = norm(d12) > dMax(1);
  end
  d = [d12; -dMax(2) + 2*dMax(2)*rand];
  
  pl.updateState(u, dt, [], d);
  pl.plotPosition();
  title(['t = ' num2str(t(i))])
  drawnow
end

end