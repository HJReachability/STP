function vehicle = augmentFlatObsBRS(vehicle, schemeData, tauIAT)
% vehicle = augmentFlatObsBRS(vehicle, schemeData, tauIAT)
%     Augments the flattened 3D obstacles with a t-IAT backward reachable set.
%     Populates the .augFlatObsBRS and .augFlatObsBRS_tau fields of vehicle.data

% Number of obstacles
numObs = length(vehicle.data.augObsFRS_tau);

% Set schemeData
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

% Compute tIAT-FRS of every base obstacle
vehicle.data.augFlatObsBRS = zeros(size(vehicle.data.augObsFRS));
for i = 1:numObs
  fprintf('Augmenting obstacle %d out of %d...\n', i, numObs)
  
  % Visualize the set every 10 obstacle augmentations
  if ~mod(i-1, 10)
    extraArgs.visualize = true;
  else
    extraArgs.visualize = false;
  end
  
  % The moving target is a union of all flattened obstacles up to time t
  extraArgs.targets = inf([schemeData.grid.N' length(tauIAT)]);
  for j = 1:length(tauIAT)
    if i-j+1 >= 1
      extraArgs.targets(:,:,:,j) = vehicle.data.cylObs3D(:,:,:,i-j+1);
    end
    
    extraArgs.targets(:,:,:,j) = min(extraArgs.targets(:,:,:,1:j), [], 4);
  end
  
  % Solve HJI PDE with the moving targets
  augFlatObsBRS = HJIPDE_solve(vehicle.data.cylObs3D(:,:,:,i), ...
    tauIAT, schemeData, 'none', extraArgs);
  vehicle.data.augFlatObsBRS(:,:,:,i) = augFlatObsBRS(:, :, :, end);
  
end

% Shift time vector
vehicle.data.augFlatObsBRS_tau = vehicle.data.augObsFRS_tau - tauIAT(end);

% Add last few obstacles
vehicle.data.augFlatObsBRS = ...
  cat(4, vehicle.data.augFlatObsBRS, augFlatObsBRS(:,:,:,2:end));
vehicle.data.augFlatObsBRS_tau = [vehicle.data.augFlatObsBRS_tau, ...
  vehicle.data.augObsFRS_tau(end-length(tauIAT)+2:end)];
end