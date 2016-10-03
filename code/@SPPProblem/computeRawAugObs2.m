function computeRawAugObs2(obj)
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

g = createGrid([-0.6; -0.6; -3*pi/2], [0.7; 0.6; pi/2], [71; 71; 71], 3);
RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, g);

%% Load CARS
fprintf('Loading CARS...\n')
load(obj.CARS_filename)

% Initialize dynamical system based on RTTRS parameters
schemeData.dynSys = Plane([0; 0; 0], ...
  RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, RTTRS.dynSys.dMaxA);

%% Compute intruder exclusive set
fprintf('Computing intruder exclusive set\n')
rawAugObs.IES = computeRawObs_IES(RTTRSdata, CARS, obj.dt, g);

%% Compute FRS
fprintf('Computing FRS of raw obstacle...\n')
schemeData.grid = g;
rawObsFRS = computeRawObs_FRS(RTTRSdata, schemeData, CARS.tau);

fprintf('Computing flat obstacle of raw obstacle FRS...\n')
tR = RTTRS.trackingRadius;
[FRS3D, g2D, FRS2D] = computeObs3D(rawObsFRS, g, obj.Rc, tR);
rawAugObs.g2D = g2D;
rawAugObs.FRS2D = FRS2D;
rawAugObs.FRS3D = FRS3D;

%% Compute BRS
fprintf('Computing flat raw obstacles\n')
tR = RTTRS.trackingRadius;
[obs3D, ~, obs2D] = computeObs3D(RTTRSdata, g, obj.Rc, tR);
rawAugObs.rawObs2D = obs2D;

fprintf('Computing BRS of flat raw obstacle...\n')
rawAugObs.BRS3D = computeRawObs_BRS(obs3D, schemeData, CARS.tau);

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

function rawObsBRS = computeRawObs_BRS(RTTRSdata, schemeData, tauIAT)
% Computes BRS or cylindrical obstacles
schemeData.uMode = 'min';
schemeData.dMode = 'min';
schemeData.tMode = 'backward';

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

rawObsBRS = HJIPDE_solve(RTTRSdata, tauIAT, schemeData,  'zero', extraArgs);

end

function IESet = computeRawObs_IES(RTTRSdata, CARS, dt, g)

schemeData.dynSys = CARS.dynSys;
schemeData.grid = g;
schemeData.uMode = 'min';
schemeData.dMode = 'min';
tau = 0:dt:2*max(CARS.tau);
data0 = RTTRSdata;

extraArgs.visualize = true;
extraArgs.deleteLastPlot = true;

IESet = HJIPDE_solve(data0, tau, schemeData, 'zero', extraArgs);
IESet = IESet(:,:,:,end);
end