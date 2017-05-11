function ctrl_reactivity(RTTRS)

RTTRS.Deriv = computeGradients(RTTRS.g, RTTRS.data);

theta = 0;
Deriv_2D = cell(3,1);
for i = 1:3
  [g2D, Deriv_2D{i}] = proj(RTTRS.g, RTTRS.Deriv{i}, [0 0 1], theta);
end

threshold = 5e-3;

%% Turn rate
W = numSign(Deriv_2D{3}, threshold);

f1 = figure;
f1.Color = 'white';
contourf(g2D.xs{1}, g2D.xs{2}, W, [-threshold threshold]);

colormap(jet)

axis equal

f1.Children.XTick = [-0.5 -0.25 0 0.25 0.5];
f1.Children.XTickLabel = {'-5', '-2.5', '0', '2.5', '5 m'};
f1.Children.YTick = [-0.5 -0.25 0 0.25 0.5];
f1.Children.YTickLabel = {'-5', '-2.5', '0', '2.5', '5 m'};
box on
grid on

savefig('Wcontrol.fig')
export_fig('Wcontrol', '-pdf')

%% Speed
V = numSign(Deriv_2D{1}, threshold);

f2 = figure;
f2.Color = 'white';
contourf(g2D.xs{1}, g2D.xs{2}, V, [-threshold threshold]);
axis equal

f2.Children.XTick = [-0.5 -0.25 0 0.25 0.5];
f2.Children.XTickLabel = {'-5', '-2.5', '0', '2.5', '5 m'};
f2.Children.YTick = [-0.5 -0.25 0 0.25 0.5];
f2.Children.YTickLabel = {'-5', '-2.5', '0', '2.5', '5 m'};
box on
grid on

savefig('Vcontrol.fig')
export_fig('Vcontrol', '-pdf')

end

function y = numSign(x, threshold)

y = zeros(size(x));
y(x > threshold) = 1;
y(x < -threshold) = -1;

end