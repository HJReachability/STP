function [cylObs3D, cylObsBRS] = computeRawObs(RTTRS_filename, tauIAT)
% [cylObs3D, cylObsBRS] = computeRawObs(RTTRS_filename, tauIAT)
%     Augments the raw obstacles (for translation on nominal trajectory)

% Load and migrate RTTRS
fprintf('Loading RTTRS...\n')
load(RTTRS_filename)

schemeData.grid = ...
  createGrid([-1; -1; -3*pi/2], [1; 1; pi/2], [101; 101; 101], 3);

RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, schemeData.grid);

% Initialize dynamical system based on RTTRS parameters
schemeData.dynSys = Plane([0; 0; 0], ...
  RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);

% Compute the sets
rawObsFRS = computeRawObs_FRS_helper(RTTRSdata, schemeData, tauIAT);
cylObs3D = computeRawObs_cylObs_helper(rawObsFRS, schemeData);
cylObsBRS = computeRawObs_BRS_helper(cylObs3D, schemeData, tauIAT);
end

function rawObsFRS = computeRawObs_FRS_helper(RTTRSdata, schemeData, tauIAT)
% Computes FRS of RTTRS
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

rawObsFRS = HJIPDE_solve(RTTRSdata, tauIAT, schemeData, 'none');
rawObsFRS = rawObsFRS(:,:,:,end);
end

function cylObs3D = computeRawObs_cylObs_helper(augObsFRS, schemeData)
% Makes 3D cylindrical obstacles from FRS of RTTRS
[~, obs2D] = proj(schemeData.grid, augObsFRS, [0 0 1]);
cylObs3D = repmat(obs2D, [1 1 schemeData.grid.N(3)]);
end

function cylObsBRS = computeRawObs_BRS_helper(cylObs3D, schemeData, tauIAT)
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

cylObsBRS = HJIPDE_solve(cylObs3D, tauIAT, schemeData, 'zero');
end

