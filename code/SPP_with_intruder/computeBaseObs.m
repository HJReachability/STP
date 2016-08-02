function vehicle = computeBaseObs(vehicle, schemeData, resetR)
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
extraArgs.SDModParams.resetR = resetR;
extraArgs.SDModParams.BRS = vehicle.data.BRS1;
extraArgs.SDModParams.tau = vehicle.data.BRS1_tau;

% Stay inside BRS1
colons = repmat({':'}, 1, schemeData.grid.dim);
extraArgs.obstacles = -vehicle.data.BRS1(colons{:}, 1:length(tau));

% Compute base obstacles
[vehicle.data.baseObs, vehicle.data.baseObs_tau] = ...
  HJIPDE_solve(obs0, tau, schemeData, 'none', extraArgs);
end