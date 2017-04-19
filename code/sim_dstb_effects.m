function sim_dstb_effects()

track_error = [0;0;0];
subSamples = 32;
for i = 2:length(nomTraj_tau)
  for s = 1:subSamples;
    % Obtain intermediate nominal trajectory points
    this_tInd = i;
    prev_tInd = i-1;

    w = s/subSamples; % weight that goes from 0 to 1
    
    nomTraj_pt = (1-w)*Q{veh}.nomTraj(:,prev_tInd) + ...
      w*Q{veh}.nomTraj(:,this_tInd);
    
    % This part indirectly takes care of the control of virtual plane
    rel_x = nomTraj_pt - Q{veh}.x;
    rel_x(1:2) = rotate2D(rel_x(1:2), -Q{veh}.x(3));
    
    deriv = eval_u(RTTRS.g, RTTRS.Deriv, rel_x);
    u = RTTRS.dynSys.optCtrl([], rel_x, deriv, 'max');
    
    % Get disturbance
    if strcmp(dstb_type, 'worst')
      d = RTTRS.dynSys.optDstb();
    elseif strcmp(dstb_type, 'const')
      d = RTTRS.dynSys.constDstb();
    elseif strcmp(dstb_type, 'uniformRand')
      d = RTTRS.dynSys.uniformDstb();
    else
      error('Unknown dstb_type!')
    end
    
    % Update state
    RTTRS.dynSys.updateState(u, obj.dt/subSamples, RTTRS.dynSys.x, d);
  end
  
  track_error = cat(2, track_error, RTTRS.dynSys.x);

end

end