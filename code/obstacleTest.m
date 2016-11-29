% Tests to see which vehicle enters the obstacle

for i = 1:length(Q)
  fprintf('i=%d\n', i)
  for j = 1:length(Q{i}.nomTraj_tau)
    if eval_u(SPPP.g2D, SPPP.staticObs, [Q{i}.nomTraj(1,j); Q{i}.nomTraj(2,j)]) < 0
      figure
      visSetIm(SPPP.g2D, SPPP.staticObs, 'k');
      hold on
      plot(Q{i}.nomTraj(1,:), Q{i}.nomTraj(2,:), 'r.-')
      keyboard
      break
    end
  end
end