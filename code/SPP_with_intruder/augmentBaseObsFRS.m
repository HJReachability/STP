function vehicle = augmentBaseObsFRS(vehicle, schemeData, tauIAT)
% vehicle = augmentBaseObsFRS(vehicle, schemeData, tauIAT)
%     Augments the base obstacles by computing the tIAT-FRS from each obstacle
%     in time, and updates the vehicle object with fields .augObsFRS and
%     .augObsFRS_tau
%
% Inputs
%     vehicle:
%         vehicle object; must contains the fields .baseObs and .baseObs_tau in
%         the .data field
%     schemeData:
%         parameters for the HJI PDE solver; must contain the fields .grid and
%         .dynSys
%     tauIAT:
%         0:dt:tIAT, maximum duration of the intruder
%
% Output
%     vehicle:
%         updated vehicle class; the fields .augObsFRS and .augObsFRS_tau will
%         be populated

% Extract the base obstacles
numObs = length(vehicle.data.baseObs_tau);

% Append each of the base obstacles by a tIAT step FRS
% Set schemeData
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

% Subtract the part of the obstacle that hits the target
extraArgs.obstacles = vehicle.data.target;

% Compute tIAT-FRS of every base obstacle
vehicle.data.augObsFRS = zeros(size(vehicle.data.baseObs));
for i = 1:numObs
  fprintf('Augmenting obstacle %d out of %d...\n', i, numObs)
  
  % Visualize the set every 10 time steps
  if ~mod(i-1, 10)
    extraArgs.visualize = true;
  else
    extraArgs.visualize = false;
  end
  
  augObsFRS = HJIPDE_solve(vehicle.data.baseObs(:, :, :, i), ...
    tauIAT, schemeData, 'none', extraArgs);
  vehicle.data.augObsFRS(:,:,:,i) = augObsFRS(:, :, :, end);
  
  % Keep first FRS at every time step
  if i == 1
    augObsFRS_beginning = augObsFRS;
  end
end

% Shift time vector
vehicle.data.augObsFRS_tau = vehicle.data.baseObs_tau + tauIAT(end);

% Add the obstacles in the beginning
vehicle.data.augObsFRS = ...
  cat(4, augObsFRS_beginning(:,:,:,1:end-1), vehicle.data.augObsFRS);
vehicle.data.augObsFRS_tau = ...
  [vehicle.data.baseObs_tau(1:(length(tauIAT)-1)) vehicle.data.augObsFRS_tau];
end