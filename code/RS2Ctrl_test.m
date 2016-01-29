function RS2Ctrl_test()
load('RS')

pl = Plane([-10 -10 0]);
pl.wMin = -1;
pl.wMax = 1;
pl.speed = 5;

U = RS2Ctrl(RS, pl.wMin, pl.wMax);

figure;
pl.plotPosition;
hold on
plot(5*cos(0:0.01:2*pi), 5*sin(0:0.01:2*pi), ':')
xlim([-11 5])
ylim([-11 5])
drawnow;

while norm(pl.getPosition) >= 5
  for i = 1:length(RS.tau)
    if eval_u(RS.g, RS.data(:,:,:,i), pl.x) <= 0
      u = interpn(RS.g.xs{1}, RS.g.xs{2}, RS.g.xs{3}, U(:,:,:,i), ...
        pl.x(1), pl.x(2), pl.x(3), 'nearest');
      break;
    end
  end
  pl.updateState(u, 0.1);
  pl.plotPosition;
  drawnow;
end

end