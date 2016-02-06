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
% Find a random point that is inside the reachable set
x = -15 + 30*rand;
y = -15 + 30*rand;
theta = 2*pi*rand;
ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);

while ival > 0
  disp(['Random value: ' num2str(ival) '; retrying...'])
  x = -10 + 20*rand;
  y = -10 + 20*rand;
  theta = 2*pi*rand;
  ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);
end
disp(['Initial value: ' num2str(ival)])

IS = [x; y; theta];
target = BRS.data(:,:,:,1);
FRS = computeFRS(FBC, IS, target);

end