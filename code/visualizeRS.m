function visualizeRS(reach, g, tau)
% visualizeRS(reach, g, tau)
%
% Visualizes a reachable set over time
%
% Inputs: reach - reachable set V(x,t)
%         g     - grid structure
%         tau   - time stamps
%
% Mo Chen, 2016-01-28

%% Compute a reachable set by default
if nargin < 1
  visualize = false;
  [reach, g, tau] = BRS(visualize);
end

%% Plot first reachable set
figure;
switch g.dim
  case 3
    h = visInitRS3(reach, g, tau);
end
drawnow;

%% Update reachable set over time
for i = 1:length(tau)
  switch g.dim
    case 3
      h = updateVisRS3(reach, g, tau, i, h);
  end
  drawnow;
end
end

%%
function h = visInitRS3(reach, g, tau)
% h = visInitRS3(reach, g, tau)
%
% Visualizes the initial 3D reachable set

h = visualizeLevelSet(g, reach(:,:,:,1), 'surface', 0, ...
  ['t = ' num2str(tau(1))]);
camlight left
camlight right
end

%%
function h = updateVisRS3(reach, g, tau, i, h)
% h = updateVisRS3(reach, g, tau, i, h)
%
% Updates the visualization of a 3D reachable set

delete(h);
h = visualizeLevelSet(g, reach(:,:,:,i), 'surface', 0, ...
  ['t = ' num2str(tau(i))]);
end