function collisionObs = getCollisionObs(g, tau, vehicle)
% collisionObs = getCollisionObs(g, tau, t, captureRadius)
%
% Given trajctory traj and corresponding time points tau, compute the
% induced obstacle of radius captureRadius at time t using linear
% interpolation in time.
%
% Mo Chen, 2014-10-13
%
% Modified by Somil to adapt to the example at hand
% Assuming the nominal trajectory is of length (tinit/tstep) + 1.
captureRadius = vehicle.capture_radius + vehicle.bubble_radius;
tinit = vehicle.t_start;
tstep = vehicle.t_step;
ip = int64((tinit- tau)/tstep) + 1; 

if(tau <= vehicle.t_start)
    collisionObs = sqrt((g.xs{1} - vehicle.x_nom(ip,1)).^2 + (g.xs{2} - vehicle.x_nom(ip,2)).^2) - captureRadius;
else
    collisionObs = 1e6*ones(g.shape);
end