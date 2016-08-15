function vehicle = computeNomTraj(vehicle, schemeData)
% Computes nominal trajectory of to be robustly tracked

% Robust trajectory tracker
dt = 0.0025;

% Save state and control histories
x = vehicle.x;
xhist = vehicle.xhist;
u = vehicle.u;
uhist = vehicle.uhist;

% Modify control bounds
vehicle.vrange = vehicle.vrange + vehicle.data.vReserved;
vehicle.wMax = vehicle.wMax + vehicle.data.wReserved;

% No disturbance when computing trajectory
d = [0; 0; 0];

% Initialize nominal trajectory
vehicle.data.nomTraj_tau = vehicle.data.BRS1_tau;
vehicle.data.nomTraj = nan(3, length(vehicle.data.BRS1_tau));
vehicle.data.nomTraj(:,1) = vehicle.x;

% Compute trajectory
small = 1e-4;
tInd = 2;
for i = 1:length(vehicle.data.BRS1_tau)-1
  while_loop = false;
  Deriv = computeGradients(schemeData.grid, vehicle.data.BRS1(:,:,:,i));
  
  while ...
      eval_u(schemeData.grid, vehicle.data.BRS1(:,:,:,i+1), vehicle.x) > small
    while_loop = true;
    deriv = eval_u(schemeData.grid, Deriv, vehicle.x);
    u = vehicle.optCtrl([], vehicle.x, deriv, 'min');
    vehicle.updateState(u, dt, vehicle.x, d);
  end
  
  if while_loop
    % Update nominal trajectory
    vehicle.data.nomTraj(:,tInd) = vehicle.x;
    tInd = tInd + 1;
  end
end

% Finalize output obstacles
vehicle.data.nomTraj_tau(tInd:end) = [];
vehicle.data.nomTraj(:,tInd:end) = [];

% Undo control bounds modification
vehicle.vrange = vehicle.vrange - vehicle.data.vReserved;
vehicle.wMax = vehicle.wMax - vehicle.data.wReserved;

% Undo state and control information
vehicle.x = x;
vehicle.xhist = xhist;
vehicle.u = u;
vehicle.uhist = uhist;

end