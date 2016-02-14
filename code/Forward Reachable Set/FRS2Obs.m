function Obs = FRS2Obs(FRS)
% Obs = FRS2Obs(FRS)
% Creates 2D obstacles Obs from the forward reachable set specified in FRS
%
% Input:  FRS - forward reachable set structure
%            .g: 3D grid structure
%            .N: number of grid points
%            .tau: time stamps
%            .data: value function
%
% Output: Obs - 2D obstacle structure
%            .g: 2D grid structure
%            .O2D: 2D value function
%            .tau: time stamps
%
% Mo Chen, 2016-02-06

% TO DO: Add Obs.O3D, the 3D value function for 3D obstacles

%% First obstacle
Obs2D = zeros(FRS.g.N(1), FRS.g.N(2), length(FRS.tau));
[g, Obs2D(:,:,1)] = proj2D(FRS.g, FRS.data(:,:,:,1), [0 0 1]);

%% The rest of the obstacles
for i = 2:length(FRS.tau)
  [~, Obs2D(:,:,i)] = proj2D(FRS.g, FRS.data(:,:,:,i), [0 0 1]);
end

%% Pack output
Obs.g = g;
Obs.O2D = Obs2D;
Obs.tau = FRS.tau;
end