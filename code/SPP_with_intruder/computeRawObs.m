function computeRawObs(RTTRS_filename, tauIAT)
% [cylObs3D, cylObsBRS] = computeRawObs(RTTRS_filename, tauIAT)
%     Augments the raw obstacles (for translation on nominal trajectory)

% Load and migrate RTTRS
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)

schemeData.grid = ...
  createGrid([-0.4; -0.5; -3*pi/2], [0.6; 0.5; pi/2], [51; 51; 51], 3);

RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, schemeData.grid);

% Initialize dynamical system based on RTTRS parameters
schemeData.dynSys = Plane([0; 0; 0], ...
  RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);

% Compute the sets
fprintf('Computing FRS of raw obstacle...\n')
rawObsFRS = computeRawObs_FRS_helper(RTTRSdata, schemeData, tauIAT);

fprintf('Computing cylObs3D of raw obstacle FRS...\n')
rawObs.cylObs3D = computeRawObs_cylObs_helper(rawObsFRS, schemeData);

fprintf('Computing BRS of cylObs3D...\n')
rawObs.cylObsBRS = ...
  computeRawObs_BRS_helper(rawObs.cylObs3D(:,:,:,end), schemeData, tauIAT);

rawObs.g = schemeData.grid;
rawObs.tauIAT = tauIAT;

save(sprintf('rawObs_%f.mat', now), 'rawObs', '-v7.3')

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

function cylObs3D = computeRawObs_cylObs_helper(augObsFRS, schemeData)
% Makes 3D cylindrical obstacles from FRS of RTTRS
cylObs3D = zeros(size(augObsFRS));
for i = 1:size(augObsFRS,4)
  [~, obs2D] = proj(schemeData.grid, augObsFRS(:,:,:,i), [0 0 1]);
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

