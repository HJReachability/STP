function computeNomTraj(obj, g)
% Computes nominal trajectory of to be robustly tracked

% Robust trajectory tracker
dt = 0.0025;

% Save state and control histories
x = obj.x;
xhist = obj.xhist;
u = obj.u;
uhist = obj.uhist;

% Modify control bounds
maxVel = obj.vrange(2);
vrange = obj.vrange;
wMax = obj.wMax;
obj.vrange = vrange + obj.vReserved;
obj.wMax = wMax + obj.wReserved;

% No disturbance when computing trajectory
d = [0; 0; 0];

% Save nominal trajectory if it already exists
oldNomTraj = obj.nomTraj;
oldNomTraj_tau = obj.nomTraj_tau;

% Initialize nominal trajectory
obj.nomTraj_tau = obj.BRS1_tau;
obj.nomTraj = nan(3, length(obj.BRS1_tau));
obj.nomTraj(:,1) = obj.x;

% Compute trajectory
small = 1e-4;
tInd = 2;
reachedTarget = false;
for i = 1:length(obj.BRS1_tau)-1
  while_loop = false;
  Deriv = computeGradients(g, obj.BRS1(:,:,:,i));
  
  while eval_u(g, obj.BRS1(:,:,:,i+1), obj.x) > small
    while_loop = true;
    deriv = eval_u(g, Deriv, obj.x);
    u = obj.optCtrl([], obj.x, deriv, 'min');
    obj.updateState(u, dt, obj.x, d);
    
    % Maximum distance per time-step
    if norm(obj.x(1:2) - obj.nomTraj(1:2,tInd-1)) > ...
        maxVel*(obj.BRS1_tau(i+1)-obj.BRS1_tau(i))
      break
    end
    
    if norm(obj.x(1:2) - obj.targetCenter(1:2)) < obj.targetRsmall + small
      reachedTarget = true;
      break
    end
  end
  
  if while_loop
    % Update nominal trajectory
    obj.nomTraj(:,tInd) = obj.x;
    tInd = tInd + 1;
  end
  
  if reachedTarget
    break
  end
end

% Delete unused indices
obj.nomTraj_tau(tInd:end) = [];
obj.nomTraj(:,tInd:end) = [];

% Pad based on FRS1_tau, if available
if ~isempty(obj.FRS1)
  indsBeforeBRS1 = find(obj.FRS1_tau < min(obj.BRS1_tau)-small);
  pad_tau = obj.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(obj.nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  obj.nomTraj_tau = [pad_tau obj.nomTraj_tau];
  obj.nomTraj = [pad_nomTraj obj.nomTraj];
end

% Attach old reachable set if needed
overwriteInds = find(oldNomTraj_tau < min(obj.nomTraj_tau) - small);
obj.nomTraj_tau = [oldNomTraj_tau(overwriteInds) obj.nomTraj_tau];
obj.nomTraj = [oldNomTraj(:,overwriteInds) obj.nomTraj];

% Undo control bounds modification
obj.vrange = vrange;
obj.wMax = wMax;

% Undo state and control information
obj.x = x;
obj.xhist = xhist;
obj.u = u;
obj.uhist = uhist;
end