function u = RBControl(x, xref, vRange, wRange, RB)
% u = RBControl(x, xref, vRange, wRange, RB): Computes control needed to track a
% nominal trajectory given the relative state x

%% Compute relative state in reference state frame
relx = x - xref;
relx(1:2) = rotate2D(relx(1:2), xref(3));
relx(3) = wrapToPi(relx(3));

%% Compute control
p = calculateCostate(RB.g, RB.P, relx);
det = p(1) * cos(relx(3)) + p(2) * sin(relx(3));

v = (det >= 0) * max(vRange) + (det < 0) * min(vRange);
w = (p(3) >= 0) * max(wRange) + (p(3) < 0) * min(wRange);

u = [v; w];

end