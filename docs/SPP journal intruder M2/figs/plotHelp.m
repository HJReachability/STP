%%
set(gca, 'fontsize', 14);

% find all lines and increase their size
set(findall(gca, 'Type', 'Line'),'LineWidth',3.5);
% set(findall(gca, 'Type', 'Line'),'Linestyle', '--', 'LineWidth', 3.5);