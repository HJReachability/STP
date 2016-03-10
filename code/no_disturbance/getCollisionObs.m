function collisionObs = getCollisionObs(g, tau, traj, t, captureRadius)
% collisionObs = getCollisionObs(g, tau, t, captureRadius)
%
% Given trajctory traj and corresponding time points tau, compute the
% induced obstacle of radius captureRadius at time t using linear
% interpolation in time.
%
% Mo Chen, 2014-10-13
%

% Figure out closest obstacles in time
ip = find(tau>t, 1, 'first');
im = ip-1;

if im < 1   % If time is earlier than earliest time stamp
    collisionObs = sqrt((g.xs{1} - traj(1,1)).^2 + (g.xs{2} - traj(1,2)).^2) - captureRadius;
elseif isempty(ip)  % If time is later than latest time stamp
    collisionObs = sqrt((g.xs{1} - traj(end,1)).^2 + (g.xs{2} - traj(end,2)).^2) - captureRadius;
else        % Otherwise, do linear interpolation in time
    collisionObsp = sqrt((g.xs{1} - traj(ip,1)).^2 + (g.xs{2} - traj(ip,2)).^2) - captureRadius;
    collisionObsm = sqrt((g.xs{1} - traj(im,1)).^2 + (g.xs{2} - traj(im,2)).^2) - captureRadius;
    
    % Time differences
    dp = tau(ip) - t;
    dm = t - tau(im);
    
    % Weights of linear interpolation
    wp = dm/(dp+dm);
    wm = dp/(dp+dm);
    collisionObs = wp*collisionObsp + wm*collisionObsm;
end
end