function computeRawObs(obj)
% [cylObs3D, cylObsBRS] = computeRawObs(RTTRS_filename, tauIAT)
%     Augments the raw obstacles (for translation on nominal trajectory)

%% Load and migrate RTTRS
fprintf('Loading RTTRS...\n')
load(obj.RTTRS_filename)

schemeData.grid = ...
  createGrid([-0.4; -0.5; -3*pi/2], [0.6; 0.5; pi/2], [51; 51; 51], 3);

RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, schemeData.grid);

%% Load CARS
fprintf('Loading CARS...\n')
load(obj.CARS_filename)

% Initialize dynamical system based on RTTRS parameters
schemeData.dynSys = Plane([0; 0; 0], ...
  RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);

% Compute the sets
fprintf('Computing FRS of raw obstacle...\n')
rawObsFRS = computeRawObs_FRS_helper(RTTRSdata, schemeData, CARS.tau);

fprintf('Computing cylObs3D of raw obstacle FRS...\n')
rawObs.cylObs3D = computeRawObs_cylObs_helper(rawObsFRS, schemeData, CARS.Rc);

fprintf('Computing BRS of cylObs3D...\n')
rawObs.cylObsBRS = ...
  computeRawObs_BRS_helper(rawObs.cylObs3D(:,:,:,end), schemeData, CARS.tau);

rawObs.g = schemeData.grid;

obj.rawObs_filename = sprintf('rawObs_%f.mat', now);
save(obj.rawObs_filename, 'rawObs', '-v7.3')

end

function rawObsFRS = computeRawObs_FRS_helper(RTTRSdata, schemeData, tauIAT)
% Computes FRS of RTTRS
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

rawObsFRS = HJIPDE_solve(RTTRSdata, tauIAT, schemeData, 'none', extraArgs);

end

function cylObs3D = computeRawObs_cylObs_helper(augObsFRS, schemeData, Rc)
% Makes 3D cylindrical obstacles from FRS of RTTRS
cylObs3D = zeros(size(augObsFRS));
g2D = proj(schemeData.grid, [], [0 0 1]);
for i = 1:size(augObsFRS,4)
  [~, obs2D] = proj(schemeData.grid, augObsFRS(:,:,:,i), [0 0 1]);
  obs2D = addCRadius(g2D, obs2D, Rc);
  cylObs3D(:,:,:,i) = repmat(obs2D, [1 1 schemeData.grid.N(3)]);
end
end

function cylObsBRS = computeRawObs_BRS_helper(cylObs3D_last, schemeData, tauIAT)
% Computes BRS or cylindrical obstacles
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

cylObsBRS = HJIPDE_solve(cylObs3D_last, tauIAT, schemeData, 'zero', extraArgs);
end

