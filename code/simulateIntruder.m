function simulateIntruder(obj)



load(obj.CARS_filename)

x0I = [0; 0; 0]; % (x, y) position is irrelevant at this point
Qintr = SPPPlane(x0I, CARS.dynSys.wMaxB, CARS.vRangeB, CARS.dMaxB);

% Index at which intruder appears
intr_tInd = 100;
figure;
plot(nomTraj(1,intr_tInd), nomTraj(2,intr_tInd), 'k.')
hold on

plotCARS(Qthis, Qintr, CARS);
extraArgsE.color = 'k';
extraArgsI = extraArgsE;
extraArgsI.color = 'r';

Qthis.plotPosition(extraArgsE)
Qintr.plotPosition(extraArgsI)

[x, y] = ginput(1);
Qintr.x(1:2) = [x; y];
drawnow;

%% Simulate
tMax = 10;
dt = obj.dt;
t = 0:dt:tMax;

for i = 2:length(t)
  % Find correct index for CARS and compute gradient
  x_rel = PlaneDubins_relState(evader.x, pursuer.x);
  tE = find_earliest_BRS_ind(CARS.g, flip(CARS.data,4), x_rel);
  tE = size(CARS.data, 4) - tE + 1;
  CARSderiv = computeGradients(CARS.g, CARS.data(:,:,:,tE));
  
  % Synthesize control and disturbance
  [uE, d] = avoid_intr_ctrl(Qthis, Qintr, CARS, CARSderiv);
  dE = -d(3:5)/2;
  uI = d(1:2);
  dI = d(3:5)/2;
  
  Qthis.updateState(uE, dt, Qthis.x, dE);
  Qintr.updateState(uI, dt, Qintr.x, dI);
  
  Qthis.plotPosition(extraArgsE)
  Qintr.plotPosition(extraArgsI)
  drawnow
end

%% Plot distance to nominal trajectory


end