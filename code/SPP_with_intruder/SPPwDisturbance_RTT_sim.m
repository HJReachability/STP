function SPPwDisturbance_RTT_sim()

tMin = -3;
dt = 0.005;
tMax = 0;
tau = tMin:dt:tMax;

% Load robust tracking reachable set
load('RTTRS.mat')

% Load path planning reachable set
load('SPPwDisturbance_RTT.mat')
Q = {Q1;Q2;Q3;Q4};

% Gradient
Deriv = computeGradients(RTTRS.g, RTTRS.data(:,:,:,end));

capture_radius = 0.1;
small = 1e-4;

% Plot targets
colors = lines(length(Q));
for veh = 1:length(Q)
  [g2D, data2D] = proj(schemeData.grid, Q{veh}.data.target, [0 0 1]);
  ht = visSetIm(g2D, data2D, colors(veh,:));
  ht.LineWidth = 3;
end

figure
for i = 1:length(tau)
  for veh = 1:1%length(Q)
    % Check if nominal trajectory has this t
    tInd = find(Q{veh}.data.nomTraj_tau > tau(i) - small & ...
      Q{veh}.data.nomTraj_tau < tau(i) + small);
    
    if ~isempty(tInd)
      %% Get optimal control
      % Our plane is vehicle A, trying to stay out of reachable set, and the 
      % reference virtual plane is vehicle B, trying to get into reachable set
      rel_x = Q{veh}.data.nomTraj(:,tInd) - Q{veh}.x;
      
      if eval_u(RTTRS.g, RTTRS.data(:,:,:,end), rel_x) <= 0
        keyboard
      end;
      
      deriv = eval_u(RTTRS.g, Deriv, rel_x);
      u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
      
      %% Get disturbance
      d = Q{veh}.GaussianDstb();

      % Update state
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
      
      % Plot capture radius
      plotDisk(Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
      
      % Plot induced obstacle
      [g2D, data2D] = ...
        proj(schemeData.grid, Q{veh}.data.cylObs3D(:,:,:,tInd), [0 0 1]);
      visSetIm(g2D, data2D, colors(veh, :));
      
      % Plot position
      Q{veh}.plotPosition(colors(veh, :));      
    end
  end
  
  if i == 1
    xlim([-1.2 1.2])
    ylim([-1.2 1.2])
  end
  title(sprintf('t = %f', tau(i)))
  drawnow;
end

end