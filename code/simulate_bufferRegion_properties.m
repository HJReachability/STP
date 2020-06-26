function simulate_bufferRegion_properties(SPPP, mode, reach_file)

if nargin < 2
  mode = 'worst';
end

if nargin < 3
  reach_file = 'toy_reach_file.mat';
end

%% Load files
if exist(SPPP.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(SPPP.CARS_filename)
else
  error('CARS file not found!')
end

if exist(SPPP.minMinBRS_filename, 'file')
  fprintf('Loading minMinBRS...\n')
  load(SPPP.minMinBRS_filename)
else
  error('minMinBRS file not found!')
end

%% Assign and check parameters
avoid_wMax = CARS.dynSys.wMaxA;
avoid_vRange = CARS.dynSys.vRangeA;

intr_wMax = CARS.dynSys.wMaxB;
intr_vRange = CARS.dynSys.vRangeB;

if avoid_wMax ~= minMinBRS.dynSys.wMaxA;
  error('CARS and minMinBRS wMaxA do not agree!')
end

if any(avoid_vRange ~= minMinBRS.dynSys.vRangeA);
  error('CARS and minMinBRS vRangeA do not agree!')
end

if intr_wMax ~= minMinBRS.dynSys.wMaxB;
  error('CARS and minMinBRS wMaxB do not agree!')
end

if any(intr_vRange ~= minMinBRS.dynSys.vRangeB);
  error('CARS and minMinBRS vRangeB do not agree!')
end

targetR = 1;

%% Target reaching file
if exist(reach_file, 'file')
  fprintf('Loading target reaching file...\n')
  load(reach_file)
else
  disp('Computing target reaching file...')
  
  grid_min = [-15; -15; -pi]; % Lower corner of computation domain
  grid_max = [15; 15; pi];    % Upper corner of computation domain
  N = [31; 31; 31];         % Number of grid points per dimension
  pdDims = 3;               % 3rd diemension is periodic
  TRRS.g = createGrid(grid_min, grid_max, N, pdDims);
  
  data0 = shapeCylinder(TRRS.g, 3, [0; 0; 0], targetR);
  tau = 0:0.05:10;
  
  schemeData.grid = TRRS.g;
  schemeData.dynSys = Plane([0, 0, 0], avoid_wMax, avoid_vRange);
  schemeData.uMode = 'min';
  TRRS.data = HJIPDE_solve(data0, tau, schemeData, 'zero');
  save(reach_file, 'TRRS', '-v7.3')
end

%% Figure parameters
f = figure;
f.Color = 'white';
f.Position = [100 100 800 600];
hold on
L = 20;
xlim(L * [-1 1])
ylim(L * [-1 1])
xlabel('x (m)', 'FontSize', 16)
ylabel('y (m)', 'FontSize', 16)
box on
grid on
axis square
title('t = 0.00', 'FontSize', 16)
eA_pos.ArrowLength = 1.5;
eA_pos.MarkerSize = 25;
eA_pos.LineStyle = '-';

set(gca, 'xtick', [-20 -10 0 10 20])
set(gca, 'ytick', [-20 -10 0 10 20])
set(gca, 'xticklabels', {'-200', '-100', '0', '100', '200'})
set(gca, 'yticklabels', {'-200', '-100', '0', '100', '200'})
set(gca, 'fontsize', 16)

colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

%% Initialize planes
% Initial positions of vehicles 1,2,3,4,I
x0s = {[0; 0; 0]; [-13.369; 13.6353; 0]; [7.9605; 16.9; 3*pi/4]; ...
  [12.3117; 0.5592; 0]; [0.0793; 1.4403; -3*pi/4]};
targets = {[-10; -10]; [-15; 0]; [-2.5; 15]; [15; 15]};
pls = cell(size(x0s));
for j = 1:length(x0s)
  if j < length(x0s)
    pls{j} = Plane(x0s{j}, avoid_wMax, avoid_vRange);
  else
    pls{j} = Plane(x0s{j}, intr_wMax, intr_vRange);
  end
end

%% Simulation parameters
switch mode
  case 'worst'
    dt = 0.05;
  case 'normal'
    dt = 0.05;
    
    for j = 1:length(targets)
      h = plotDisk(targets{j}, targetR, 'color', colors{j}, 'linewidth', 2);
    end
    
  otherwise
    error('Unknown mode!')
end

tMax = 10;
t = 0:dt:tMax;
small = 0.01;

%% Plot reachable sets
if ~exist(SPPP.RBR_filename, 'file')
  SPPP.computeRBR(true);
end
load(SPPP.RBR_filename)

hCARS = cell(length(pls)-1, 1);
hRBR  = cell(length(pls)-1, 1);

for j = 1:length(pls)
  eA_pos.Color = colors{j};
  pls{j}.plotPosition(eA_pos);
  if j < length(pls)
    hCARS{j} = plotCARS(pls{j}, pls{end}, CARS);
  end
end

CARSderiv = computeGradients(CARS.g, CARS.data(:,:,:,end));

vri = 2; % vehicle reaching intruder
hRBR{vri} = plotRBR(pls{vri}, pls{end}, RBR);

switch mode
  case 'worst'
    leg = legend([pls{1}.hpxpy, pls{1}.hpxpyhist, hCARS{1}, hRBR{vri}], ...
      {'Pos. and heading', 'Trajectory', 'Avoid region', ...
      '(Relative) buffer region'}, 'FontSize', 16', 'Location', 'SouthEast');
  case 'normal'
    leg = legend([h, pls{1}.hpxpy, pls{1}.hpxpyhist, hCARS{1}, hRBR{vri}], ...
      {'Target', 'Pos. and heading', 'Trajectory', 'Avoid region', ...
      '(Relative) buffer region'}, 'FontSize', 16', 'Location', 'SouthEast');
end

us = cell(size(pls));
savefig(sprintf('%s_%s_%d.fig', mfilename, mode, vri-1));
export_fig(sprintf('%s_%s_%d', mfilename, mode, vri-1), '-pdf')

leg.Visible = 'off';
hRBR{vri}.Visible = 'off';

for i = 2:length(t)
  for j = 1:length(pls)-1
    switch mode
      case 'worst'
        if j == vri
          [us{j},us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
        elseif j == 1
          us{j} = reach_ctrl(pls{j}, TRRS, targets{j});
        else
          us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS, CARSderiv);
        end
        
      case 'normal'
        x_rel = PlaneDubins_relState(pls{j}.x, pls{end}.x);
        if eval_u(CARS.g, CARS.data(:,:,:,end), x_rel) < small
          us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS, CARSderiv);
        else
          us{j} = reach_ctrl(pls{j}, TRRS, targets{j});
        end
        
        [~, us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
    end
    
  end
  
  % Update states
  for j = 1:length(pls)
    pls{j}.updateState(us{j}, dt);
  end
  
  % Update which vehicle tries to reach the intruder
  x_rel = PlaneDubins_relState(pls{vri}.x, pls{end}.x);
  if eval_u(CARS.g, CARS.data(:,:,:,end), x_rel) < small
    vri = vri + 1;
    hRBR{vri} = plotRBR(pls{vri}, pls{end}, RBR);
    
%     keyboard
    savefig(sprintf('%s_%s_%d.fig', mfilename, mode, vri-1));
    export_fig(sprintf('%s_%s_%d', mfilename, mode, vri-1), '-pdf')
    
    hRBR{vri}.Visible = 'off';
  end
  
  % Update plot
  for j = 1:length(pls)
    pls{j}.plotPosition(eA_pos);
    
    if j < length(pls)
      delete(hCARS{j})
      hCARS{j} = plotCARS(pls{j}, pls{end}, CARS);
    end
  end
  
  title(sprintf('t = %.2f\n', t(i)))
  drawnow
end

savefig(sprintf('%s_%s_%d.fig', mfilename, mode, vri));
export_fig(sprintf('%s_%s_%d', mfilename, mode, vri), '-pdf')
end

function [u, uI] = reach_intr_ctrl(evader, pursuer, RBR, minMinBRS)
% [u, uI] = reach_intr_ctrl(evader, pursuer, RBR, minMinBRS)
%     Computes worst-case control to reach intruder

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

tE = find_earliest_BRS_ind(RBR.g, flip(RBR.data,4), x_rel);
tE = size(RBR.data, 4) - tE + 1;
RBRderiv = computeGradients(RBR.g, RBR.data(:,:,:,tE));

p = eval_u(RBR.g, RBRderiv, x_rel);
u = minMinBRS.dynSys.optCtrl([], x_rel, p, 'min');

uI = minMinBRS.dynSys.optDstb([], x_rel, p, 'min');
uI = uI(1:2);

end

function u = reach_ctrl(vehicle, TRRS, target_pos)
% u = reach_target_ctrl(vehicle, TRRS, target_pos)
%     Computes target reaching control

x_rel = vehicle.x;
x_rel(1:2) = x_rel(1:2) - target_pos;

if eval_u(TRRS.g, TRRS.data(:,:,:,1), x_rel) < 0
  u = [0; 0];
  return
end

tE = find_earliest_BRS_ind(TRRS.g, flip(TRRS.data,4), x_rel);
tE = size(TRRS.data, 4) - tE + 1;
TRRSderiv = computeGradients(TRRS.g, TRRS.data(:,:,:,tE));

p = eval_u(TRRS.g, TRRSderiv, x_rel);
u = vehicle.optCtrl([], vehicle.x, p, 'min');
end

function h = plotRBR(evader, pursuer, RBR)
% h = plotRBR(evader, pursuer, RBR, color)
%     Plots min min reachable set from max min reachable set
[g2D,data2D] = proj(RBR.g, RBR.data(:,:,:,end), [0 0 1], pursuer.x(3));
RBRdata = rotateData(g2D, data2D,evader.x(3), [1 2], []);
RBR_gShift = shiftGrid(g2D, evader.x(1:2));

eA_min_min.LineStyle = ':';
eA_min_min.LineWidth = 2;
h = visSetIm(RBR_gShift, RBRdata, 'b', 0, eA_min_min);
end