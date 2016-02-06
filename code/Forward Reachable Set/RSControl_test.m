function RSControl_test(recompute)
% RSControl_test(recompute)
%
% Tests the Plane.RSControl function
%
% Set recompute to true to recompute reachable sets
%
% Mo Chen, 2016-02-02

addpath(genpath('../../../UTM/code/platoon'))

if nargin < 1
  recompute = false;
end

%% Compute backwards reachable set
BRS_file = ['test_data/' mfilename '_BRS.mat'];
if exist(BRS_file, 'file') && ~recompute
  load(BRS_file)
else
  BRS = computeBRS();
  save(BRS_file, 'BRS')
end

N = 5;
for i = 1:N
  RSControl_test_single(BRS);
end
end

function RSControl_test_single(BRS)
%% Initialize Plane
% Find a random point that is inside the reachable set
x = -15 + 30*rand;
y = -15 + 30*rand;
theta = 2*pi*rand;
ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);

while ival > 0
  disp(['Random value: ' num2str(ival) '; retrying...'])
  x = -10 + 20*rand;
  y = -10 + 20*rand;
  theta = 2*pi*rand;
  ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);
end
disp(['Initial value: ' num2str(ival)])

pl = Plane([x y theta]);
pl.wMin = -1;
pl.wMax = 1;
pl.speed = 5;

%% Plot initial conditions
figure;
pl.plotPosition;
hold on
[g2D, data2D] = proj2D(BRS.g, BRS.data(:,:,:,1), [0 0 1], 0);
contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0]);
xlim([-15 15])
ylim([-15 15])
drawnow;

%% Head to target
t0 = -3;
dt = 0.1;
tMax = 0;
t = t0:dt:tMax;
  
for i = 1:length(t)
  u = pl.RSControl(BRS, t(i));
  pl.updateState(u, dt);
  pl.plotPosition;
  drawnow;
  if eval_u(BRS.g, BRS.data(:,:,:,1), pl.x) <= 0
    break;
  end
end
end