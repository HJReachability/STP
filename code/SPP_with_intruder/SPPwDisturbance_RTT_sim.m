function SPPwDisturbance_RTT_sim(Q)

tMin = -3;
dt = 0.1;
tMax = 0;
tau = tMin:dt:tMax;

% Gradient
Deriv = computeGradients(RTTRS.g, RTTRS.data(:,:,:,end));

for i = 1:length(tau)
  tInd = cell(length(Q),1);
  for veh = 1:length(Q)
    % Check if BRS1 has been computed for this t
    tInd = find(Q{veh}.data.baseObs_tau > tau(i) - small & ...
      Q{veh}.data.baseObs_tau < tau(i) + small);
    
    if ~isempty(tInd{veh})
      %% Get optimal control
      % Our plane is vehicle A, trying to stay out of reachable set, and the 
      % reference virtual plane is vehicle B, trying to get into reachable set
      x_ref = Q{veh}.data.nomTraj(:,tInd);
      rel_x = x_ref - Q{veh}.x;
      deriv = eval_u(RTTRS.g, Deriv, rel_x);
      u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
      
      %% Get disturbance
      d = [0; 0; 0];
      Q{veh}.updateState(u, dt, Q{veh}.x, d);
    end
    
  end
end

end