function [g2D, flatObs2D, cylObs3D] = flatAugObs(g, obs, obs_tau, rc)
% [g2D, flatObs2D, cylObs3D] = flatAugObs(g, obs, obs_tau, rc)
%     Flattens the obstacles given by g and obs, augments the flatted obstacle
%     with the capture radius rc, and then create 3D cylindrical obstacles out
%     of the 2D obstacles

%% Parameters for adding capture radius
g2D = proj(g, [], [0 0 1]);

%% Initialize augmented-capture-radius flat obstacles
flatObs2D = zeros([g2D.N' length(obs_tau)]);
cylObs3D = zeros([g.N' length(obs_tau)]);

for i = 1:length(obs_tau)
  [~, obs2D] = proj(g, obs(:,:,:,i), [0 0 1]);
  flatObs2D(:,:,i) = addCRadius(g2D, obs2D, rc);
  cylObs3D(:,:,:,i) = repmat(flatObs2D(:,:,i), [1 1 g.N(3)]);
end
end