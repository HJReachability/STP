function dstb_effects_obstacles(SPPPs, veh_num)
% dstb_effects_obstacles(SPPPs)
% SPPPs - cell vector of SPPProblem objects

if nargin < 2
  veh_num = 17;
end

dist_fig = figure;

obs_fig = SPPPs{1}.plotSetup('SF', false);

small = 1e-2;
for i = 1:length(SPPPs)
  load(SPPPs{i}.NI_sim_filename)
  
  %% Obstacles
  figure(obs_fig)
  plot(Q{veh_num}.xhist(1,:), Q{veh_num}.xhist(2,:))
  if i == 1
    hold on
  end
  
  %% Distances
  vehicles = 1:length(Q);
  vehicles(vehicles == veh_num) = [];
  Qref = Q{veh_num};
  tau = Qref.tau;
  dists = inf(length(vehicles), length(tau));
  
  for j = 1:length(vehicles)
    Qother = Q{vehicles(j)};
    other_tau_inds = Qother.tau > min(tau)-small & Qother.tau < max(tau)+small;
    ref_tau_inds = tau > min(Qother.tau)-small & tau < max(Qother.tau)+small;
    
    dists(j, ref_tau_inds) = sqrt( ...
      (Qother.xhist(1,other_tau_inds) - Qref.xhist(1,ref_tau_inds)).^2 + ...
      (Qother.xhist(2,other_tau_inds) - Qref.xhist(2,ref_tau_inds)).^2);
  end
  
  min_dist = min(dists, [], 1);
  
  figure(dist_fig)
  plot(tau, min_dist)
  if i == 1
    hold on
    plot([min(tau) max(tau)], [1 1], 'r:')
  end  
end



end