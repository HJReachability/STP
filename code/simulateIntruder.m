function simulateIntruder(SPPP, Qthis)

load(SPPP.CARS_filename)

% Index at which intruder appears
intr_tInd = 100;
figure;
x0 = Qthis.xhist(:,intr_tInd);
Qnew = Plane(x0, Qthis.wMax, Qthis.vrange, Qthis.dMax);
plot(x0(1), x0(2), 'k.')
hold on

% Intruder
x0I = [0; 0; x0(3)-0.95*pi]; % (x, y) position is irrelevant at this point
Qintr = SPPPlane(x0I, CARS.dynSys.wMaxB, CARS.dynSys.vRangeB, ...
  CARS.dynSys.dMaxB);

plotCARS(Qnew, Qintr, CARS);
extraArgsE.Color = 'k';
extraArgsE.LineWidth = 2;
extraArgsI = extraArgsE;
extraArgsI.Color = 'r';

Qnew.plotPosition(extraArgsE)

xlim([x0(1)-30 x0(1)+30])
ylim([x0(2)-30 x0(2)+30])

[~, phist_orig] = Qthis.getPosition;
plot(phist_orig(1,:), phist_orig(2,:), 'k.-')
drawnow;

% [x, y] = ginput(1)
x = 411.5268;
y = 159.9875;

% Initialize intruder and new instance of plane
Qintr.x(1:2) = [x; y];
Qintr.xhist(1:2) = [x; y];
Qintr.plotPosition(extraArgsI)
drawnow;

%% Simulate
tMax = 10;
dt = SPPP.dt;
t = 0:dt:tMax;

for i = 2:length(t)
  % Find correct index for CARS and compute gradient
  x_rel = PlaneDubins_relState(Qnew.x, Qintr.x);
  tE = find_earliest_BRS_ind(CARS.g, flip(CARS.data,4), x_rel);
  tE = size(CARS.data, 4) - tE + 1;
  CARSderiv = computeGradients(CARS.g, CARS.data(:,:,:,tE));
  
  % Synthesize control and disturbance
  [uE, d] = avoid_intr_ctrl(Qnew, Qintr, CARS, CARSderiv);
  dE = -d(3:5)/2;
  uI = [d(1); 0];d(1:2);
  dI = d(3:5)/2;
  
  Qnew.updateState(uE, dt, Qnew.x, dE);
  Qintr.updateState(uI, dt, Qintr.x, dI);
  
  Qnew.plotPosition(extraArgsE)
  Qintr.plotPosition(extraArgsI)
  drawnow
end

%% Plot distance to nominal trajectory
[~, phist_intr] = Qintr.getPosition;
[~, phist_new] = Qnew.getPosition;
phist_orig = phist_orig(:, intr_tInd+1:intr_tInd+length(t));

dist_new = 10*sqrt((phist_new(1,:) - phist_intr(1,:)).^2 + ...
  (phist_new(2,:) - phist_intr(2,:)).^2);

dist_orig = 10*sqrt((phist_orig(1,:) - phist_intr(1,:)).^2 + ...
  (phist_orig(2,:) - phist_intr(2,:)).^2);

figure;
h_rc = plot(t, 10*SPPP.Rc*ones(size(t)), 'k--');
hold on
h_new = plot(t, dist_new, 'b*-');

h_orig = plot(t, dist_orig, 'r.-');
xlabel('time (s)')
ylabel('distance to intruder (m)')
legend([h_rc h_new h_orig], {'Collision', 'Avoiding', 'Not avoiding'}, ...
  'Location', 'best')

end