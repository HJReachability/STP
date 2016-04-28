function IS = randISinRS(BRS)
% IS = randISinRS(BRS)
%
% Creates an initial condition that is contained in the earliest reachable set
%
% Input:  BRS - backwards reachable set
% Output: IS  - initial state (3D column vector)
%
% Mo Chen, 2016-02-06

%% Randomly pick a state in the grid and check if it's inside the reachable set
x = BRS.g.min(1) + (BRS.g.max(1)-BRS.g.min(1))*rand;
y = BRS.g.min(2) + (BRS.g.max(2)-BRS.g.min(2))*rand;
theta = 2*pi*rand;
ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);

%% Retry until a suitable state is found
while ival > 0
  disp(['Random value: ' num2str(ival) '; retrying...'])
  x = -10 + 20*rand;
  y = -10 + 20*rand;
  theta = 2*pi*rand;
  ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);
end
disp(['Initial value: ' num2str(ival)])

%% Pack output
IS = [x; y; theta];
end