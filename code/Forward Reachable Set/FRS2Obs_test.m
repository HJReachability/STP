function FRS2Obs_test(recompute)
% FRS2Obs_test(recompute)
% Tests the FRS2Obs function
%
% Set recompute to true to recompute the backwards reachable set, the
% feedback control law, and the forwards reachable set

if nargin < 1
  recompute = false;
end

%% Compute backwards reachable set
BRS_file = ['test_data/' mfilename '_BRS.mat'];
if exist(BRS_file, 'file') && ~recompute
  load(BRS_file)
else
  BRS = computeBRS();
  save(BRS_file, 'BRS')
end

%% Compute feedback control from backwards reachable set
FBC_file = ['test_data/' mfilename '_FBC.mat'];
if exist(FBC_file, 'file') && ~recompute
  load(FBC_file)
else
  FBC = RS2Ctrl(BRS);
  save(FBC_file, 'FBC')
end

%% Compute forward reachable set
FRS_file = ['test_data/' mfilename '_FRS.mat'];
if exist(FRS_file, 'file') && ~recompute
  load(FRS_file)
else
  IS = randISinRS(BRS);
  target = BRS.data(:,:,:,1);
  t0 = latestRS(IS, BRS);
  FRS = computeFRS(FBC, IS, target, t0);
  save(FRS_file, 'FRS')
end

%% Compute obstacles
Obs = FRS2Obs(FRS);

%% Plot
colors = jet(length(Obs.tau));
figure;
contour(Obs.g.xs{1}, Obs.g.xs{2}, Obs.O2D(:,:,1), [0 0], ...
  'color', colors(1,:))
hold on
for i = 2:length(Obs.tau)
  contour(Obs.g.xs{1}, Obs.g.xs{2}, Obs.O2D(:,:,i), [0 0], ...
    'color', colors(i,:));
end

end