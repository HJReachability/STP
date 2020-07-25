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
  load('SPPProblem_worst_toy/CARS2.mat');
else
  SPPP.computeCARS
%   error('CARS file not found!')
end

if exist(SPPP.minMinBRS_filename, 'file')
  fprintf('Loading minMinBRS...\n')
  load(SPPP.minMinBRS_filename)
else
  SPPP.computeMinMinBRS;
%   error('minMinBRS file not found!')
end

%% Assign and check parameters
avoid_wMax = CARS.dynSys.wMaxA;
avoid_vRange = CARS.dynSys.vRangeA;

intr_wMax = CARS.dynSys.wMaxB;
intr_vRange = CARS.dynSys.vRangeB;

if avoid_wMax ~= minMinBRS.dynSys.wMaxA
  error('CARS and minMinBRS wMaxA do not agree!')
end

if any(avoid_vRange ~= minMinBRS.dynSys.vRangeA)
  error('CARS and minMinBRS vRangeA do not agree!')
end

if intr_wMax ~= minMinBRS.dynSys.wMaxB
  error('CARS and minMinBRS wMaxB do not agree!')
end

if any(intr_vRange ~= minMinBRS.dynSys.vRangeB)
  error('CARS and minMinBRS vRangeB do not agree!')
end

targetR = 1;

%% Target reaching file
if exist(reach_file, 'file')
  fprintf('Loading target reaching file...\n')
  load(reach_file)
else
  disp('Computing target reaching file...')
  
  grid_min = [-50; -50; -pi]; % Lower corner of computation domain
  grid_max = [50; 50; pi];    % Upper corner of computation domain
  N = [201; 201; 36];         % Number of grid points per dimension
  pdDims = 3;               % 3rd diemension is periodic
  TRRS.g = createGrid(grid_min, grid_max, N, pdDims);
  
  data0 = shapeCylinder(TRRS.g, 3, [0; 0; 0], targetR);
  tau = 0:0.05:30;
  
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
xlim([-10 40])
ylim([-20 30])
xlabel('x (m)', 'FontSize', 16)
ylabel('y (m)', 'FontSize', 16)
box on
grid on
axis square
title('t = 0.00', 'FontSize', 16)
eA_pos.ArrowLength = 1.5;
eA_pos.MarkerSize = 25;
eA_pos.LineStyle = '-';

set(gca, 'xtick', [-10 0 10 20 30 40])
set(gca, 'ytick', [-20 -10 0 10 20 30])
set(gca, 'xticklabels', {'-100', '0', '100', '200', '300', '400'})
set(gca, 'yticklabels', {'-200', '-100', '0', '100', '200', '300'})
set(gca, 'fontsize', 16)

colors = {'b', 'r', [0 0.5 0], 'm', 'k'};

%% Initialize planes
% Initial positions of vehicles 1,2,3,4,I
% For worst case
% x0s = {[0; 0; 0]; [-13.369; 13.6353; 0]; [7.9605; 16.9; 3*pi/4]; ...
%   [12.3117; 0.5592; 0]; [0.0793; 1.4403; -3*pi/4]};
% For normal caseS{{
x0s = {[0; 0; 0]; ...
       [-2; 12; 0]; ...
       [12; 16.9; 3*pi/4]; ...
       [12.3117; 0.1; 0]; ...
       [0.0793; 1.4403; -3*pi/4]};

vMax = CARS.dynSys.vRangeA(2);
 
x02st = [-0.8676; 7.978; atan2(-0.9996, 0.0292)];
x0s{2} = x02st - 1.9 * [vMax*cos(x02st(3)); vMax*sin(x02st(3)); 0];

x03st = [6.19; 15.46; atan2(-0.8977, -0.4405)];
x0s{3} = x03st - 5.75 * [vMax*cos(x03st(3)); vMax*sin(x03st(3)); 0];

x04st = [13.85; 9.61; atan2(-0.5155, -0.8569)];
x0s{4} = x04st - 11.5 * [vMax*cos(x04st(3)); vMax*sin(x04st(3)); 0];
% 
% targets = {[-10; -10]; ...
%            [-15; 0]; ...
%            [-2.5; 15]; ...
%            [15; 15]};

targets = cell(4,1);
for i = 1:4
  targets{i} = x0s{i}(1:2) + ...
    (11 + targetR) * [vMax*cos(x0s{i}(3)); vMax*sin(x0s{i}(3))];
end
         
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
    
    for j = 1:length(targets)
      h = plotDisk(targets{j}, targetR, 'color', colors{j}, 'linewidth', 2);
      plot([x0s{j}(1) targets{j}(1)], [x0s{j}(2) targets{j}(2)], ...
        ':', 'color', colors{j}, 'linewidth', 2.5);
    end    
  case 'normal'
    dt = 0.05;
    
    for j = 1:length(targets)
      h = plotDisk(targets{j}, targetR, 'color', colors{j}, 'linewidth', 2);
    end
    
  otherwise
    error('Unknown mode!')
end

tMax = 30;
tin = 10;
t = 0:dt:tMax;
small = 0.02;

%% Plot reachable sets
if ~exist(SPPP.RBR_filename, 'file')
  SPPP.computeRBR(true);
else
  load(SPPP.RBR_filename) 
end


hCARS = cell(length(pls)-1, 1);
hRBR  = cell(length(pls)-1, 1);

for j = 1:length(pls)
  eA_pos.Color = colors{j};
  pls{j}.plotPosition(eA_pos);
  if j < length(pls)
    hCARS{j} = plotCARS(pls{j}, pls{end}, CARS);
  end
end

CARS2deriv = computeGradients(CARS2.g, CARS2.data(:,:,:,end));

% vri = 2; % vehicle reaching intruder
vri = 2; % vehicles >= this number do not need to avoid intruder yet
hRBR{vri} = plotRBR(pls{vri}, pls{end}, RBR);

switch mode
  case 'worst'
%     leg = legend([pls{1}.hpxpy, pls{1}.hpxpyhist, hCARS{1}, hRBR{vri}], ...
%       {'Pos. and heading', 'Trajectory', 'Avoid region', ...
%       '(Relative) buffer region'}, 'FontSize', 16', 'Location', 'SouthEast');
  leg = legend([h, pls{1}.hpxpy, pls{1}.hpxpyhist, hCARS{1}, hRBR{vri}], ...
    {'Target', 'Pos. and heading', 'Trajectory', 'Avoid region', ...
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

tagged = false(4,1);
tagged(1) = true;

mkdir(sprintf('%s/%s_sim/%s_%s', SPPP.folder, mode, mfilename, mode), '-png')
for i = 2:length(t)
  if t(i) <= tin
    for j = 1:length(pls)-1
% %       switch mode
%         case 'worst'
%           if j == vri
%             [us{j},us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
%           elseif j == 1
%             us{j} = reach_ctrl(pls{j}, TRRS, targets{j});
%           else
%             us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS, CARSderiv);
%           end
%           [~,us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
          if j >= vri
            us{j} = [vMax; 0];
          else
            us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS, CARS2);
%             fprintf('u{%d} = (%.1f, %.1f)\n', j, us{j}(1), us{j}(2));
          end
          
%         case 'normal'
%           x_rel = PlaneDubins_relState(pls{j}.x, pls{end}.x);
%           if ~tagged(j) && eval_u(CARS.g, CARS.data(:,:,:,end), x_rel) < small
%               tagged(j) = true;
%           end
% 
%           if tagged(j) && t(i) <= tin
%             us{j} = avoid_intr_ctrl(pls{j}, pls{end}, CARS);
%           else
%             us{j} = reach_ctrl(pls{j}, TRRS, targets{j});
%           end
% 
%           if t(i) <= tin
%               [~, us{end}] = reach_intr_ctrl(pls{vri}, pls{end}, RBR, minMinBRS);
%           else
%               us{end} = [0; 0];
%           end
%       end

    end
    [~,us{end}] = avoid_intr_ctrl(pls{vri}, pls{end}, CARS, CARS2);
    us{end} = us{end}(1:2);
    
    % Update which vehicle tries to reach the intruder
    x_rel = PlaneDubins_relState(pls{vri}.x, pls{end}.x);
    if eval_u(CARS.g, CARS.data(:,:,:,end), x_rel) < small
      vri = vri + 1;
  %     hRBR{vri} = plotRBR(pls{vri}, pls{end}, RBR);

  %     keyboard
      savefig(sprintf('%s_%s_%d.fig', mfilename, mode, vri-1));
      export_fig(sprintf('%s_%s_%d', mfilename, mode, vri-1), '-pdf')

      hRBR{vri}.Visible = 'off';
    end
  
  else
    for j = 1:length(pls)-1
      us{j} = reach_ctrl(pls{j}, TRRS, targets{j});
    end
    us{end} = [0; 0];
  end
  
  % Update states
  for j = 1:length(pls)
    pls{j}.updateState(us{j}, dt);
  end
 
  % Update plot
  for j = 1:length(pls)
    if t(i) == tin
      [~, phist] = pls{j}.getPosition;
      plot(phist(1,:), phist(2,:), '-', 'color', colors{j}, ...
        'linewidth', 0.5);
      eA_pos.LineStyle = '--';
    end
    pls{j}.plotPosition(eA_pos);
    
%     if j < length(pls)
%       delete(hCARS{j})
%       hCARS{j} = plotCARS(pls{j}, pls{end}, CARS);
%     end
  end
  
  title(sprintf('t = %.2f\n', t(i)))
  drawnow
  export_fig(sprintf('%s/%s_sim/%s_%s_%d', SPPP.folder, mode, mfilename, mode, i), '-png')
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