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

small = 0.1;

x01 = [0; 0; 0];
x02 = [-11; 12.7; 0];
pl1 = Plane(x01, avoid_wMax, avoid_vRange);
pl2 = Plane(x02, avoid_wMax, avoid_vRange);
% pl3 = Plane(x03, avoid_wMax, avoid_vRange);

x0I = [3; 0.79; -3*pi/4];
plI = Plane(x0I, intr_wMax, intr_vRange);

dt = 0.1;
tMax = 10;
t = 0:dt:tMax;

figure;
[g2D, data2D] = proj(CARS.g, CARS.data(:,:,:,end), [0 0 1], x0I(3));
visSetIm(g2D, data2D, 'r');

hold on
xlim([-20 20])
ylim([-20 20])
extraArgs.ArrowLength = 1;
extraArgs.MarkerSize = 15;
pl1.plotPosition(extraArgs);
plI.plotPosition(extraArgs);

% Second vehicle
if ~exist(obj.RBR_filename, 'file')
  obj.computeRBR;
end
load(obj.RBR_filename)

[g2D, data2D] = proj(RBR.g, RBR.data(:,:,:,end), [0 0 1], x0I(3));
RBRdata = rotateData(g2D, data2D, x0I(3), [1 2], []);
RBR_gShift = shiftGrid(g2D, x02(1:2));

visSetIm(RBR_gShift, RBRdata, 'b')
pl2.plotPosition(extraArgs);

CARSderiv = computeGradients(CARS.g, CARS.data(:,:,:,end));

for i = 2:length(t)
  % Vehicle 1 avoids intruder
  x1I = PlaneDubins_relState(pl1.x, plI.x);
  p1 = eval_u(CARS.g, CARSderiv, x1I);
  u1 = CARS.dynSys.optCtrl([], x1I, p1, 'max');
  pl1.updateState(u1, dt);
  
  % Vehicle 2 and intruder try to collide
  x2I = PlaneDubins_relState(pl2.x, plI.x);
  
  if eval_u(CARS.g, CARS.data(:,:,:,end), x2I) < small
    keyboard;
  end
  
  tE = find_earliest_BRS_ind(RBR.g, flip(RBR.data,4), x2I);
  tE = size(RBR.data, 4) - tE + 1;
  RBRderiv = computeGradients(RBR.g, RBR.data(:,:,:,tE));

  [g2D, data2D] = proj(RBR.g, RBR.data(:,:,:,tE), [0 0 1], x2I(3));
  RBRdata = rotateData(g2D, data2D, pl2.x(3), [1 2], []);
  RBR_gShift = shiftGrid(g2D, pl2.x(1:2));

  if i > 2
    delete(hRBR)
    delete(h)
  end
  
  hRBR = visSetIm(RBR_gShift, RBRdata, 'k');
  
  p2 = eval_u(minMinBRS.g, RBRderiv, x2I);
  u2 = minMinBRS.dynSys.optCtrl([], x2I, p2, 'min');
  pl2.updateState(u2, dt);
  
  uI = minMinBRS.dynSys.optDstb([], x2I, p2, 'min');
  uI = uI(1:2);
  plI.updateState(uI, dt);
  
  pl1.plotPosition(extraArgs);
  pl2.plotPosition(extraArgs);
  plI.plotPosition(extraArgs);
  
  % max min set 2I
  [g2D, data2D] = proj(CARS.g, CARS.data(:,:,:,end), [0 0 1], x2I(3));
  CARSdata = rotateData(g2D, data2D, pl2.x(3), [1 2], []);
  CARS_gShift = shiftGrid(g2D, pl2.x(1:2));
  
  h = visSetIm(CARS_gShift, CARSdata);
  title(sprintf('t=%.2f\n', t(i)))
  drawnow
end

end