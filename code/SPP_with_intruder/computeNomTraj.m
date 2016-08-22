function vehicle = computeNomTraj(vehicle, schemeData)
% Computes nominal trajectory of to be robustly tracked

% Robust trajectory tracker
dt = 0.0025;

% For readability
g = schemeData.grid;
BRS1_tau = vehicle.data.BRS1_tau;

% Save state and control histories
x = vehicle.x;
xhist = vehicle.xhist;
u = vehicle.u;
uhist = vehicle.uhist;

% Modify control bounds
maxVel = vehicle.vrange(2);
vehicle.vrange = vehicle.vrange + vehicle.data.vReserved;
vehicle.wMax = vehicle.wMax + vehicle.data.wReserved;

% No disturbance when computing trajectory
d = [0; 0; 0];

% Save nominal trajectory if it already exists
nomTrajBefore = vehicle.data.nomTraj;
nomTraj_tauBefore = vehicle.data.nomTraj_tau;

% Initialize nominal trajectory
nomTraj_tau = BRS1_tau;
nomTraj = nan(3, length(BRS1_tau));
nomTraj(:,1) = vehicle.x;

% Compute trajectory
small = 1e-4;
tInd = 2;
for i = 1:length(BRS1_tau)-1
  while_loop = false;
  Deriv = computeGradients(g, vehicle.data.BRS1(:,:,:,i));
  
  while eval_u(g, vehicle.data.BRS1(:,:,:,i+1), vehicle.x) > small
    while_loop = true;
    deriv = eval_u(g, Deriv, vehicle.x);
    u = vehicle.optCtrl([], vehicle.x, deriv, 'min');
    vehicle.updateState(u, dt, vehicle.x, d);
    
    % Maximum distance per time-step
    if norm(vehicle.x(1:2) - nomTraj(1:2,tInd-1)) > ...
        maxVel*(BRS1_tau(i+1)-BRS1_tau(i))
      break
    end
  end
  
  if while_loop
    % Update nominal trajectory
    nomTraj(:,tInd) = vehicle.x;
    tInd = tInd + 1;
  end
end

% Delete unused indices
nomTraj_tau(tInd:end) = [];
nomTraj(:,tInd:end) = [];

% Pad based on FRS1_tau, if available
if isfield(vehicle.data, 'FRS1')
  indsBeforeBRS1 = find(vehicle.data.FRS1_tau < min(BRS1_tau)-small);
  pad_tau = vehicle.data.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  nomTraj_tau = [pad_tau nomTraj_tau];
  nomTraj = [pad_nomTraj nomTraj];
end

% Attach old reachable set if needed
overwriteInds = find(nomTraj_tauBefore < min(nomTraj_tau) - small);
vehicle.data.nomTraj_tau = [nomTraj_tauBefore(overwriteInds) nomTraj_tau];
vehicle.data.nomTraj = [nomTrajBefore(:,overwriteInds) nomTraj];

% Undo control bounds modification
vehicle.vrange = vehicle.vrange - vehicle.data.vReserved;
vehicle.wMax = vehicle.wMax - vehicle.data.wReserved;

% Undo state and control information
vehicle.x = x;
vehicle.xhist = xhist;
vehicle.u = u;
vehicle.uhist = uhist;
end