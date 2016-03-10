function tracking_test()

addpath(genpath('.'))
addpath(genpath('C:\Users\Mo\Documents\UTM\code\platoon'))
%% Problem parameters
tradius = 0.1;
speed = 0.75;
uMax = 0.6;
dMax = [0; 0; 0];

%% Compute or reachable set
filename = ['BRS_' num2str(tradius) '_' num2str(speed) '_' num2str(uMax) ...
  '_' num2str(dMax(1)) num2str(dMax(2)) num2str(dMax(3)) '.mat'];
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
[g2D, data2D] = proj2D(BRS.g, BRS.data(:,:,:,1), [0 0 1]);
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'r')

xlim([-1 1])
ylim([-1 1])
title('t = 0')
drawnow;

dt = 0.1;
tMax = 10;
t = 0:dt:tMax;

for i = 1:length(t)
  x = plv.x;
  x(3) = wrapTo2Pi(x(3));
  
  if eval_u(BRS.g, BRS.data(:,:,:,1), x) <= 0
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
load('RB.mat')
vRange = [0.5 1];
wRange = [-1 1];

%% Follow trajectory
pl = Plane(IS);

% Plot initial setup
figure;
pl.plotPosition(); % Initial position
hold on

% Target set
[g2D, data2D] = proj2D(BRS.g, BRS.data(:,:,:,1), [0 0 1]);
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'r')

xlim([-1 1])
ylim([-1 1])
title('t = 0')
drawnow;

for i = 1:length(t)
  % Relative state
  xv = plv.xhist(:, i);
  x = pl.x;
  relx = x - xv;
  relx(1:2) = rotate2D(relx(1:2), xv(3));
  relx(3) = wrapTo2Pi(relx(3));
  
  % Bubble control
  u = RBControl(x, vRange, wRange, RB);
  
  pl.updateState(u, dt);
  pl.plotPosition();
  drawnow
end

end