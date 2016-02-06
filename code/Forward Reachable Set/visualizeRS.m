function visualizeRS(RS, mode, numPlots)
% visualizeRS(RS, mode, numPlots)
%
% Visualizes a reachable set over time
%
% Inputs: RS       - Reachable set structure
%           .data: reachable set V(x,t)
%           .g   : grid structure
%           .tau : time stamps
%
%         mode     - use 'single' to view a single plot that changes over time
%                    use 'multi' to view multiple subplots
%
%         numPlots - number of subplots to view (ignored for 'single' mode)
%
% Mo Chen, 2016-02-06

%% Compute a reachable set by default
if nargin < 1
  visualize = false;
  RS = BRS(visualize);
end

if nargin < 2
  mode = 'single';
end

%% Reachable set over time in the same plot
if strcmp(mode, 'single')
  % Plot first reachable set
  figure;
  switch RS.g.dim
    case 3
      h = visSingleRS3(RS);
  end
  drawnow;
  
  % Update reachable set over time
  for i = 1:length(RS.tau)
    switch RS.g.dim
      case 3
        h = updateVisRS3(RS, i, h);
    end
    drawnow;
  end
  return
end

%% Reachable sets in subplots
if strcmp(mode, 'multi')
  if nargin < 3
    numPlots = 4;
  end
  
  spC = ceil(sqrt(numPlots));
  spR = ceil(numPlots/spC);
  
  plotInds = (1:(numPlots-2))*length(RS.tau)/(numPlots-1);
  plotInds = [1 ceil(plotInds) length(RS.tau)];
  
  figure;
  for i = 1:numPlots
    subplot(spC, spR, i)
    visSingleRS3(RS, plotInds(i));
  end
  return
end

error('Unknown visualization mode!')
end

%% Visualize a single 3 dimensional reachable set over time
function h = visSingleRS3(RS, i)
% h = visInitRS3(reach, g, tau)
%
% Visualizes the initial 3D reachable set

if nargin < 2
  i = 1;
end

h = visualizeLevelSet(RS.g, RS.data(:,:,:,i), 'surface', 0, ...
  ['t = ' num2str(RS.tau(i))]);
axis(RS.g.axis)
camlight left
camlight right
end

%% Update a 3 dimensional reachable set
function h = updateVisRS3(RS, i, h)
% h = updateVisRS3(reach, g, tau, i, h)
%
% Updates the visualization of a 3D reachable set

delete(h);
h = visualizeLevelSet(RS.g, RS.data(:,:,:,i), 'surface', 0, ...
  ['t = ' num2str(RS.tau(i))]);
end