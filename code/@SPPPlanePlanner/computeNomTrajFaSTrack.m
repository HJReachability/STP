function computeNomTrajFaSTrack(obj, SPPP_obj, g, SPPP_folder, veh)
% SPPPlane.computeNomTraj(g)
%     Computes nominal trajectory to be robustly tracked

small = 1e-4;

% Modify control bounds
pMax = SPPP_obj.pMax;
dynSys = Plane2D(obj.x, pMax(1), pMax(2));

% Set extraArgs
extraArgs.uMode = 'min';
extraArgs.visualize = true;
extraArgs.projDim = [1 1];

if g.dim > 2
  for i = 1:g.dim - 2
      extraArgs.projDim = [extraArgs.projDim 0];
  end
end
extraArgs.subSamples = 32;

if ispc
  folder = sprintf('%s\\%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir %s', folder));
else
  folder = sprintf('%s/%s_%d', SPPP_folder, mfilename, veh);
  system(sprintf('mkdir -p %s', folder));
end

extraArgs.fig_filename = sprintf('%s/', folder);

% Compute trajectory
[nomTraj, nomTraj_tau] = ...
  computeOptTraj(g, obj.BRS1, obj.BRS1_tau, dynSys, extraArgs);

% Pad based on FRS1_tau, if available
if isempty(obj.FRS1)
  % Update nominal trajectory if it doesn't exist
  obj.nomTraj_tau = nomTraj_tau;
  obj.nomTraj = nomTraj;  
else
  indsBeforeBRS1 = find(obj.FRS1_tau < min(obj.BRS1_tau)-small);
  pad_tau = obj.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  nomTraj_tau = [pad_tau nomTraj_tau];
  nomTraj = [pad_nomTraj nomTraj];
  
  obj.nomTraj_AR_tau = nomTraj_tau;
  obj.nomTraj_AR = nomTraj;    
end

end