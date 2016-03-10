function u = RBControl(x, vRange, wRange, RB)

p = calculateCostate(RB.g, RB.P, x);
det = p(1) * cos(x(3)) + p(2) * sin(x(3));

v = (det >= 0) * max(vRange) + (det < 0) * min(vRange);
w = (p(3) >= 0) * max(wRange) + (p(3) < 0) * min(wRange);

u = [v; w];

end