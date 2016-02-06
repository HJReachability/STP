function RS2Ctrl_test(recompute)
% Tests the RS2Ctrl function
%
% Set recompute to true to recompute reachable sets rather than loading
%
% Mo Chen, 2016-02-02

if nargin < 1
  recompute = false;
end

% Add platooning project path
addpath(genpath('../../../UTM/code/platoon'))

%% Compute backwards reachable set
BRS_file = ['test_data/' mfilename '_BRS.mat'];
if exist(BRS_file, 'file') && ~recompute
  load(BRS_file)
else
  BRS = computeBRS();
  save(BRS_file, 'BRS')
end

%% Compute feedback control and test using a Plane object
FBC = RS2Ctrl(BRS);

N = 10;
for i = 1:N
  RS2Ctrl_test_single(BRS, FBC);
end
end

function RS2Ctrl_test_single(BRS, FBC)
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
for i = 1:length(BRS.tau)
  u = interpn(BRS.g.xs{1}, BRS.g.xs{2}, BRS.g.xs{3}, FBC.U(:,:,:,i), ...
    pl.x(1), pl.x(2), pl.x(3), 'nearest');
  
  pl.updateState(u, 0.1);
  pl.plotPosition;
  drawnow;
  
  if eval_u(BRS.g, BRS.data(:,:,:,1), pl.x) <= 0
    break;
  end
end
end