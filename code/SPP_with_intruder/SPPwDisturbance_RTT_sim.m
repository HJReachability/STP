function SPPwDisturbance_RTT_sim(RTTRS_file, RTT_file, save_png, save_fig)

if nargin < 3
  save_png = true;
end

if nargin < 4
  save_fig = false;
end

tMin = -3;
dt = 0.01;
tMax = 0;
tau = tMin:dt:tMax;

% Load robust tracking reachable set
load(RTTRS_file)

% Load path planning reachable set
load(RTT_file)
Q = {Q1;Q2;Q3;Q4};

% Gradient of RTTRS
Deriv = computeGradients(RTTRS.g, RTTRS.data(:,:,:,end));

capture_radius = 0.1;
small = 1e-4;

% Plot targets
f = figure;
colors = lines(length(Q));
plotTargetSets(Q, schemeData, colors)

hc = cell(length(Q), 1);
ho = cell(length(Q), 1);

% For saving graphics
folder = sprintf('%s_%f', mfilename, now);
system(sprintf('mkdir %s', folder));

for i = 1:length(tau)
  fprintf('t = %f\n', tau(i))
  for veh = 1:length(Q)
    % Check if nominal trajectory has this t
    tInd = find(Q{veh}.data.nomTraj_tau > tau(i) - small & ...
      Q{veh}.data.nomTraj_tau < tau(i) + small);
    
    if ~isempty(tInd)
      %% Get optimal control
      % Our plane is vehicle A, trying to stay out of reachable set, and the 
      % reference virtual plane is vehicle B, trying to get into reachable set
      rel_x = Q{veh}.data.nomTraj(:,tInd) - Q{veh}.x;
      rel_x(1:2) = rotate2D(rel_x(1:2), -Q{veh}.x(3));
     
      deriv = eval_u(RTTRS.g, Deriv, rel_x);
      u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
      
      %% Get disturbance
      d = Q{veh}.uniformDstb();

      % Update state
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
      
      % Plot capture radius
      if isempty(hc{veh})
        hc{veh} = plotDisk( ...
          Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
      else
        [~, hc{veh}.XData, hc{veh}.YData] = plotDisk( ...
          Q{veh}.getPosition, capture_radius, '-', 'color', colors(veh,:));
      end
      
      % Plot induced obstacle for vehicles 1 to 3
      if veh < length(Q)
        [g2D, data2D] = ...
          proj(schemeData.grid, Q{veh}.data.cylObs3D(:,:,:,tInd), [0 0 1]);
        if isempty(ho{veh})
          ho{veh} = visSetIm(g2D, data2D, colors(veh, :));
          ho{veh}.LineStyle = '--';
        else
          ho{veh}.ZData = data2D;
        end
      end
      
      % Plot position
      Q{veh}.plotPosition(colors(veh, :));      
    end
  end
  
  xlim([-1.2 1.2])
  ylim([-1.2 1.2])
  
  title(sprintf('t = %f', tau(i)))
  drawnow;
  if save_png
    export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')
  end
  
  if save_fig
    savefig(f, sprintf('%s/%d', folder, i), 'compact')
  end
end

end