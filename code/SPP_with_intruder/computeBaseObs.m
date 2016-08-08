function vehicle = computeBaseObs(vehicle, schemeData, method, params)
% vehicle = computeBaseObs(vehicle, schemeData, resetR)
%     Computes the induced base obstacles by vehicle according to BRS1 and the
%     centralized controller scheme; populates the .baseObs and .baseObs_tau
%     fields of vehicle.data
%
% Inputs
%     vehicle:
%         vehicle object; should have the .data field populated with .BRS1 and
%         .BRS1_tau
%     schemeData:
%         parameters for the HJI PDE solver; should contain the fields .grid and
%         .dynSys
%     resetR:
%         minimum size for the reachable set during evolution; if the reachable
%         set goes below this size, the reachable set will be propagated
%         "maximally" until the size becomes large enough
%
% Output
%     vehicle:
%         updated vehicle object with .baseObs and .baseObs_tau populated in the
%         vehicle.data field

tau = vehicle.data.BRS1_tau(1:end-1);

if strcmp(method, 'RTT')
  % Robust trajectory tracker
  Deriv = computeGradients(schemeData.grid, vehicle.data.BRS1);
  
  % Modify control bounds
  vehicle.vrange = vehicle.vrange + vehicle.data.vReserved;
  vehicle.wMax = vehicle.wMax + vehicle.data.wReserved;
  
  % No disturbance when computing trajectory
  d = [0; 0; 0];
  
  % Initialize obstacle
  vehicle.data.baseObs_tau = vehicle.data.BRS1_tau(1:end-1);
  vehicle.data.baseObs = ...
    zeros([schemeData.grid.N' length(vehicle.data.BRS1_tau)-1]);
  
  % Initialize nominal trajectory
  vehicle.data.nomTraj = zeros(3, length(vehicle.data.BRS1_tau)-1);
  
  % Rotate and shift the robust trajectory tracking reachable set to the vehicle
  % state, then subtract the target set
  RTTRS = rotateData(schemeData.grid, params.RTTRS, vehicle.x(3), [1 2], 3);
  RTTRS = shiftData(schemeData.grid, RTTRS, vehicle.x([1 2]), [1 2]);
  vehicle.data.baseObs(:,:,:,1) = RTTRS;
  
  % Compute trajectory
  small = 1e-3;
  for i = 1:length(vehicle.data.BRS1_tau)-1
    while ...
        eval_u(schemeData.grid, vehicle.data.BRS1(:,:,:,i+1), vehicle.x) > small
      deriv = eval_u(schemeData.grid, ...
        {Deriv{1}(:,:,:,i); Deriv{2}(:,:,:,i); Deriv{3}(:,:,:,i)}, vehicle.x);
      u = vehicle.optCtrl([], vehicle.x, deriv, 'min');
      vehicle.updateState(u, ...
        vehicle.data.BRS1_tau(i+1)-vehicle.data.BRS1_tau(i), vehicle.x, d);
    end
    
    % Rotate and shift the robust trajectory tracking reachable set to the vehicle
    % state
    RTTRS = rotateData(schemeData.grid, params.RTTRS, vehicle.x(3), [1 2], 3);
    RTTRS = shiftData(schemeData.grid, RTTRS, vehicle.x([1 2]), [1 2]);
    vehicle.data.baseObs(:,:,:,i) = RTTRS;
    
    % Update nominal trajectory
    vehicle.data.nomTraj(:,i) = vehicle.x;
  end
  
  % Undo control bounds modification
  vehicle.vrange = vehicle.vrange - vehicle.data.vReserved;
  vehicle.wMax = vehicle.wMax - vehicle.data.wReserved;
  
  % Undo state information
  vehicle.x = vehicle.xhist(:,1);
  vehicle.xhist = vehicle.xhist(:,1);
  
elseif strcmp(method, 'CC')
  % Set schemeData
  schemeData.dMode = 'max';
  schemeData.tMode = 'forward';
  
  % Set computation time (ignore last time step, when vehicle will be at target)
  tau = vehicle.data.BRS1_tau(1:end-1);
  
  % Create a small obstacle around current vehicle
  obs0 = genBaseObs0(schemeData.grid, vehicle.x, resetR);
  
  % Set extraArgs for visualization
  extraArgs.visualize = true;
  
  % Set extraArgs for centralized controller
  extraArgs.SDModFunc = @minIslandSize_SDFunc;
  extraArgs.SDModParams.resetR = params.resetR;
  extraArgs.SDModParams.BRS = vehicle.data.BRS1;
  extraArgs.SDModParams.tau = vehicle.data.BRS1_tau;
  
  % Stay inside BRS1
  colons = repmat({':'}, 1, schemeData.grid.dim);
  extraArgs.obstacles = -vehicle.data.BRS1(colons{:}, 1:length(tau));
  
  % Compute base obstacles
  [vehicle.data.baseObs, vehicle.data.baseObs_tau] = ...
    HJIPDE_solve(obs0, tau, schemeData, 'none', extraArgs);
end
end