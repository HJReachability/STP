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