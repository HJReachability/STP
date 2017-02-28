close all; clear; 

% Open the setup fig file
h1 = openfig('init_setup.fig','reuse'); % open figure
ax{1} = gca; % get handle to axes of figure
% Extract the legned for this figure
h{1} = gcf; 
axesObjs{1} = get(h{1}, 'Children');  %axes handles

% Open the cc trajectory file
h2 = openfig('cc_traj.fig','reuse'); % open figure
ax{2} = gca; % get handle to axes of figure
% Extract the legned for this figure
h{2} = gcf; 
axesObjs{2} = get(h{2}, 'Children');  %axes handles

% Open the LRC trajectory file
h3 = openfig('lrc_traj.fig','reuse'); % open figure
ax{3} = gca; % get handle to axes of figure
% Extract the legned for this figure
h{3} = gcf; 
axesObjs{3} = get(h{3}, 'Children');  %axes handles

% Open the RTT trajectory file
h4 = openfig('rtt_traj.fig','reuse'); % open figure
ax{4} = gca; % get handle to axes of figure
% Extract the legned for this figure
h{4} = gcf; 
axesObjs{4} = get(h{4}, 'Children');  %axes handles

% Create new figure
f = figure; 
pos = get(f, 'position');
set(f, 'position', [pos(1) pos(2) 560 600]);

% Title strings
titstr = {'Initial setup', 'Centralized control', 'Least restrictive control', 'Robust trajectory tracking'};

for i=1:4
  %create and get handle to the subplot axes
  s{i} = subplot(2,2,i);
  %get handle to all the children in the figure
  child{i} = axesObjs{i}(2);  % get handle to axes of figure
%   leg{i} = findobj(axesObjs{i}, 'Tag', 'legend');
  leg{i} = axesObjs{i}(1);
%   figs{i} = get(ax{i},'children');
  %copy children to new parent axes i.e. the subplot axes
  copyobj(child{i}.Children,s{i});
  if i==1
%     temp = findobj(gca, 'Type', 'Line', 'Linestyle', '-');
%     temp = flip(temp);
%     legend(temp, leg{i}.String{:});
    obj1 = findobj(gca, 'Type', 'quiver');
    obj1 = flip(obj1);
    legend(obj1, leg{i}.String{:});
  else
    obj1 = findobj(gca, 'Type', 'quiver');
    obj2 = findobj(gca, 'Type', 'contour');
    obj3 = findobj(gca, 'Type', 'Line', 'Linestyle', ':');
    legend([obj1(1); obj2(1); obj2(2); obj3(1)], 'Positions, Headings', 'Danger Zones', 'Targets', 'Trajectories');
    %legend(leg{i}.String{:});
  end
  % Set x and y ticks
  set(gca,'YTickMode','manual');
  set(gca,'YTick',[-0.5, 0, 0.5]);
  set(gca,'XTickMode','manual');
  set(gca,'XTick',[-0.5, 0, 0.5]);
  % set x and y limits
  % Set Fontsize and axes
  set(gca, 'Fontsize', 14);
  axis equal;
  box on;
  set(findall(gca, 'Type', 'Line'),'LineWidth',2.5);
  % Title setting
  title(titstr{i});
end

% Move the bottom two subplots a bit higher
p = get(s{3}, 'pos');
p(2) = 0.15;
set(s{3}, 'pos', p);

p = get(s{4}, 'pos');
p(2) = 0.15;
set(s{4}, 'pos', p);