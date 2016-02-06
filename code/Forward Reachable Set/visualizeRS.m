function visualizeRS(RS)
% visualizeRS(reach, g, tau)
%
% Visualizes a reachable set over time
%
% Inputs: RS - Reachable set structure
%           .data: reachable set V(x,t)
%           .g   : grid structure
%           .tau : time stamps
%
% Mo Chen, 2016-01-31

%% Compute a reachable set by default
if nargin < 1
  visualize = false;
  RS = BRS(visualize);
end

%% Plot first reachable set
figure;
switch RS.g.dim
  case 3
    h = visInitRS3(RS);
end
drawnow;

%% Update reachable set over time
for i = 1:length(RS.tau)
  switch RS.g.dim
    case 3
      h = updateVisRS3(RS, i, h);
  end
  drawnow;
end
end

%%
function h = visInitRS3(RS)
% h = visInitRS3(reach, g, tau)
%
% Visualizes the initial 3D reachable set

h = visualizeLevelSet(RS.g, RS.data(:,:,:,1), 'surface', 0, ...
  ['t = ' num2str(RS.tau(1))]);
axis(RS.g.axis)
camlight left
camlight right
end

%%
function h = updateVisRS3(RS, i, h)
% h = updateVisRS3(reach, g, tau, i, h)
%
% Updates the visualization of a 3D reachable set

delete(h);
h = visualizeLevelSet(RS.g, RS.data(:,:,:,i), 'surface', 0, ...
  ['t = ' num2str(RS.tau(i))]);
end