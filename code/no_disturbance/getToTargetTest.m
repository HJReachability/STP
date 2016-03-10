clear all

addpath(genpath('/Users/mochen72/Desktop/UTM/code/platoon'))
visualize = false;
RS = BRS(visualize);

pl = Plane([-10 -10 0]);
pl.wMin = -1;
pl.wMax = 1;
pl.speed = 5;

figure;
pl.plotPosition;
hold on
plot(5*cos(0:0.01:2*pi), 5*sin(0:0.01:2*pi), ':')
xlim([-11 5])
ylim([-11 5])
drawnow;

while norm(pl.getPosition) >= 5
  u = pl.RSControl(RS);
  pl.updateState(u, 0.1);
  pl.plotPosition;
  drawnow;
end