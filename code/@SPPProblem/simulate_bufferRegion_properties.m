function simulate_bufferRegion_properties(obj)

%% Load files
if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

if exist(obj.minMinBRS_filename, 'file')
  fprintf('Loading minMinBRS...\n')
  load(obj.minMinBRS_filename)
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

%% Simulation parameters
dt = 0.05;
tMax = 10;
t = 0:dt:tMax;
small = 0.01;

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

%% Initialize planes
x0s = {[0; 0; 0]; [-11.224; 12.7; 0]; [9.2; 16.9; 3*pi/4]; [16; 5; 0]; ...
  [3.127; 0.505; -3*pi/4]}; % Initial positions of vehicles 1,2,3,4,I
pls = cell(size(x0s));
for j = 1:length(x0s)
  if j < length(x0s)
    pls{j} = Plane(x0s{j}, avoid_wMax, avoid_vRange);
  else
    pls{j} = Plane(x0s{j}, intr_wMax, intr_vRange);
  end
end

%% Plot reachable sets
if ~exist(obj.RBR_filename, 'file')
  obj.computeRBR;
end
load(obj.RBR_filename)

hCARS = cell(length(pls)-1, 1);
hRBR  = cell(length(pls)-1, 1);
colors = cat(1, lines(length(pls)-1), [0 0 0]);
for j = 1:length(pls)
  eA_pos.Color = colors(j,:);
  pls{j}.plotPosition(eA_pos);
  if j < length(pls)
    hCARS{j} = plotCARS(pls{j}, pls{end}, CARS);
  end
end

CARSderiv = computeGradients(CARS.g, CARS.data(:,:,:,end));

vri = 2; % vehicle reaching intruder
hRBR{vri} = plotRBR(pls{vri}, pls{end}, RBR);

leg = legend([pls{1}.hpxpy, pls{1}.hpxpyhist, hCARS{1}, hRBR{vri}], ...
  {'Pos. and heading', 'Trajectory', 'Avoid region', ...
  '(Relative) buffer region'}, 'FontSize', 16', 'Location', 'SouthEast');

us = cell(size(pls));

savefig(sprintf('%s_%d.fig', mfilename, vri-1));
export_fig(sprintf('%s_%d', mfilename, vri-1), '-pdf')

leg.Visible = 'off';
hRBR{vri}.Visible = 'off';

for i = 2:length(t)
  for j = 1:length(pls)-1
    if j == vri
      [us{j}, us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
    else
      us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS, CARSderiv);
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
    
    savefig(sprintf('%s_%d.fig', mfilename, vri-1));
    export_fig(sprintf('%s_%d', mfilename, vri-1), '-pdf')

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

savefig(sprintf('%s_%d.fig', mfilename, vri));
export_fig(sprintf('%s_%d', mfilename, vri), '-pdf')
end

function u = avoid_intr_ctrl(evader, pursuer, CARS, CARSderiv)
% u = avoid_intr_ctrl(evader, pursuer, CARS, CARSderiv)
%     Computes avoidance control

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

if any(x_rel' >= CARS.g.max) || any(x_rel' <= CARS.g.min)
  u = [0; 0];
else
  p = eval_u(CARS.g, CARSderiv, x_rel);
  u = CARS.dynSys.optCtrl([], x_rel, p, 'max');
end
end

function [u, uI] = reach_intr_ctrl(evader, pursuer, RBR, minMinBRS)
% [u, uI] = reach_intr_ctrl(evader, pursuer, RBR, minMinBRS)
%     Computes worst-case controlt o reach intruder

x_rel = PlaneDubins_relState(evader.x, pursuer.x);

tE = find_earliest_BRS_ind(RBR.g, flip(RBR.data,4), x_rel);
tE = size(RBR.data, 4) - tE + 1;
RBRderiv = computeGradients(RBR.g, RBR.data(:,:,:,tE));

p = eval_u(RBR.g, RBRderiv, x_rel);
u = minMinBRS.dynSys.optCtrl([], x_rel, p, 'min');

uI = minMinBRS.dynSys.optDstb([], x_rel, p, 'min');
uI = uI(1:2);

end

function h = plotCARS(evader, pursuer, CARS)
% h = plotCARS(evader, pursuer, CARS)
%     Plots max min reachable set
x_rel = PlaneDubins_relState(evader.x, pursuer.x);

[g2D, data2D] = proj(CARS.g, CARS.data(:,:,:,end), [0 0 1], x_rel(3));
CARSdata = rotateData(g2D, data2D, evader.x(3), [1 2], []);
CARS_gShift = shiftGrid(g2D, evader.x(1:2));

eA_max_min.LineStyle = '--';
eA_max_min.LineWidth = 2;
h = visSetIm(CARS_gShift, CARSdata, 'r', 0, eA_max_min);
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