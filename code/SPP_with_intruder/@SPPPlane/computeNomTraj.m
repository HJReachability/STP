function computeNomTraj(obj, g)
% Computes nominal trajectory of to be robustly tracked

% Save nominal trajectory if it already exists
oldNomTraj = obj.nomTraj;
oldNomTraj_tau = obj.nomTraj_tau;

folder = sprintf('nomTraj_%f', now);
system(sprintf('mkdir %s', folder));

% Compute trajectory
vrange = obj.vrange + obj.vReserved;
wMax = obj.wMax + obj.wReserved;
dynSys = Plane(obj.x, wMax, vrange);

extraArgs.uMode = 'min';
extraArgs.visualize = true;
extraArgs.projDim = [1 1 0];
extraArgs.save_png = true;

[obj.nomTraj, obj.nomTraj_tau] = ...
  computeOptTraj(g, obj.BRS1, obj.BRS1_tau, dynSys, extraArgs);

% Pad based on FRS1_tau, if available
if ~isempty(obj.FRS1)
  indsBeforeBRS1 = find(obj.FRS1_tau < min(obj.BRS1_tau)-small);
  pad_tau = obj.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(obj.nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  obj.nomTraj_tau = [pad_tau obj.nomTraj_tau];
  obj.nomTraj = [pad_nomTraj obj.nomTraj];
end

% Attach old reachable set if needed
keepInds = find(oldNomTraj_tau < min(obj.nomTraj_tau) - small);
obj.nomTraj_tau = [oldNomTraj_tau(keepInds) obj.nomTraj_tau];
obj.nomTraj = [oldNomTraj(:,keepInds) obj.nomTraj];
end