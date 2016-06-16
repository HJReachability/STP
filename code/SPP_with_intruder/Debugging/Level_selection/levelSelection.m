index = 120; 
P = extractCostates(g, data(:, :, :, index));
dataBRS = find(data(:, :, :, index) <= 0);
P1 = P{1}(dataBRS); 
P2 = P{2}(dataBRS);
P3 = P{3}(dataBRS);


gradV_overall = P{1}*g.dx(1) + P{2}*g.dx(2) + P{3}*g.dx(3);
gradV_BRS = P1*g.dx(1) + P2*g.dx(2) + P3*g.dx(3);

minV_overall = min(min(min(abs(gradV_overall))));
minV_BRS = min(min(min(abs(gradV_BRS))));
