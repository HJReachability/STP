function FRS = computeFRS_test(recompute)
% FRS = computeFRS_test(recompute)
%
% Tests the computeFRS function
%
% Set recompute to true to recompute the backwards reachable set and the
% feedback control law
%
% Mo Chen, 2016-02-02

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
IS = randISinRS(BRS);
target = BRS.data(:,:,:,1);
t0 = latestRS(IS, BRS);
FRS = computeFRS(FBC, IS, target, t0);

visualizeRS(FRS, 'multi', 9);
end