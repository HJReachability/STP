function vehicle = flatAugObs(vehicle, schemeData, rc, obs_field)
% vehicle = flatAugObs(vehicle, schemeData, rc, obs_field)
%     flattens and augments with capture radius to the obstacle in obs_field

vehicle.data.cylObs3D_tau = vehicle.data.(sprintf('%s_tau', obs_field));

vehicle.data.cylObs3D = ...
  flatAugObs_helper(schemeData.grid, vehicle.data.(obs_field), rc);

end

function cylObs3D = flatAugObs_helper(g, obs, rc)
% [g2D, flatObs2D, cylObs3D] = flatAugObs(g, obs, obs_tau, rc)
%     Flattens the obstacles given by g and obs, augments the flatted obstacle
%     with the capture radius rc, and then create 3D cylindrical obstacles out
%     of the 2D obstacles

%% Parameters for adding capture radius
g2D = proj(g, [], [0 0 1]);
obsSize = size(obs);

%% Initialize augmented-capture-radius flat obstacles
flatObs2D = zeros([g2D.N' obsSize(end)]);
cylObs3D = zeros([g.N' obsSize(end)]);

for i = 1:obsSize(end)
  [~, obs2D] = proj(g, obs(:,:,:,i), [0 0 1]);
  flatObs2D(:,:,i) = addCRadius(g2D, obs2D, rc);
  cylObs3D(:,:,:,i) = repmat(flatObs2D(:,:,i), [1 1 g.N(3)]);
end
end