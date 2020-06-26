function x_rel = PlaneDubins_relState(evader_x, pursuer_x)

x_rel = pursuer_x - evader_x;
x_rel(1:2) = rotate2D(x_rel(1:2), -evader_x(3));
x_rel(3) = wrapToPi(x_rel(3));

end

