function computeRawAugObs(obj)
% computeRawAugObs(obj)
%     Augments the raw obstacles (for translation on nominal trajectory)

if exist(obj.rawAugObs_filename, 'file')
  fprintf(['The rawObs file %s already exists. Skipping rawObs ' ...
  'computation.\n'], obj.rawAugObs_filename)
  return
end

%% Load and migrate RTTRS
fprintf('Loading RTTRS...\n')
load(obj.RTTRS_filename)

g = createGrid([-0.5; -0.6; -3*pi/2], [0.7; 0.6; pi/2], [65; 65; 65], 3);
RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, g);

%% Load CARS
fprintf('Loading CARS...\n')
load(obj.CARS_filename)

% Initialize dynamical system based on RTTRS parameters
schemeData.dynSys = Plane([0; 0; 0], ...
  RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);

% Compute the sets
fprintf('Computing FRS of raw obstacle...\n')
schemeData.grid = g;
rawObsFRS = computeRawObs_FRS(RTTRSdata, schemeData, CARS.tau);

fprintf('Computing cylObs3D of raw obstacle FRS...\n')
tR = RTTRS.trackingRadius;
[FRS3D, g2D, obs2D] = computeObs3D(rawObsFRS, g, obj.Rc, tR);
rawAugObs.g2D = g2D;
rawAugObs.FRS2D = obs2D;

fprintf('Computing BRS of obs3D...\n')
rawAugObs.datas = computeObs3D_BRS(FRS3D, schemeData, CARS.tau);
rawAugObs.g = g;

obj.rawAugObs_filename = sprintf('%s_%f.mat', mfilename, now);
save(obj.rawAugObs_filename, 'rawAugObs', '-v7.3')

SPPP = obj;
save(obj.this_filename, 'SPPP', '-v7.3')

end

function rawObsFRS = computeRawObs_FRS(RTTRSdata, schemeData, tauIAT)
% Computes FRS of RTTRS
schemeData.uMode = 'max';
schemeData.dMode = 'max';
schemeData.tMode = 'forward';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

rawObsFRS = HJIPDE_solve(RTTRSdata, tauIAT, schemeData, 'none', extraArgs);

end

function [obs3D, g2D, obs2D] = computeObs3D(augObsFRS, g, Rc, tR)
% Makes 3D cylindrical obstacles from FRS of RTTRS
obs3D = zeros(size(augObsFRS));
[g2D, obs2D] = proj(g, augObsFRS, [0 0 1]);

for i = 1:size(augObsFRS,4)
  obs2D(:,:,i) = addCRadius(g2D, obs2D(:,:,i), Rc+tR);
  obs3D(:,:,:,i) = repmat(obs2D(:,:,i), [1 1 g.N(3)]);
end
end

function rawAugObs_datas = computeObs3D_BRS(obs3D, schemeData, tauIAT)
% Computes BRS or cylindrical obstacles
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

rawAugObs_datas = cell(length(tauIAT), 1);

for i = 1:length(tauIAT)
  rawAugObs_datas{i} = HJIPDE_solve(obs3D(:,:,:,i), tauIAT, schemeData, ...
    'zero', extraArgs);
end

end

