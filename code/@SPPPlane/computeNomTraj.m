function computeNomTraj(obj, g, SPPP_folder, veh)
% SPPPlane.computeNomTraj(g)
%     Computes nominal trajectory of to be robustly tracked

small = 1e-4;

% Modify control bounds
nom_vrange = obj.vrange + obj.vReserved;
nom_wMax = obj.wMax + obj.wReserved;
dynSys = Plane(obj.x, nom_wMax, nom_vrange);

% Set extraArgs
extraArgs.uMode = 'min';
extraArgs.visualize = true;
extraArgs.projDim = [1 1 0];
extraArgs.subSamples = 8;

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
if ~isempty(obj.FRS1)
  indsBeforeBRS1 = find(obj.FRS1_tau < min(obj.BRS1_tau)-small);
  pad_tau = obj.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  nomTraj_tau = [pad_tau nomTraj_tau];
  nomTraj = [pad_nomTraj nomTraj];
end

% Update object fields
if isempty(obj.nomTraj)
  % Update nominal trajectory if it doesn't exist
  obj.nomTraj_tau = nomTraj_tau;
  obj.nomTraj = nomTraj;
else
  % Update nominal trajectory after replanning if nominal trajectory exists
  obj.nomTraj_AR_tau = nomTraj_tau;
  obj.nomTraj_AR = nomTraj;  
end
end