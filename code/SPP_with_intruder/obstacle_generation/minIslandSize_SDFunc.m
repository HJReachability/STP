function schemeData = minIslandSize_SDFunc(schemeData, i, tau, data, ~, paramsIn)
% schemeData = minIslandSize_SDFunc(schemeData, i, ~, data, ~, paramsIn)
%     Modifies schemeData to use optimal control given by BRS, or use maximal
%     FRS controller (max_u max_d) if the island sizes becomes too small
% 

colons = repmat({':'}, 1, schemeData.grid.dim);

% [~, ~, rs] = findIslands(schemeData.grid, data(colons{:}, i-1), 0);
% 
% %% If smallest island is less than resetR, then use max max calculation
% for j = 1:length(rs)
%   if min(rs{j}) < paramsIn.resetR
%     if isfield(schemeData, 'uIn')
%       schemeData = rmfield(schemeData, 'uIn');
%     end
%     schemeData.uMode = 'max';
%     return
%   end
% end

%% Otherwise, compute optimal control and then plug it into schemeData.uIn
tNow = tau(i-1);
[~, ii] = max(paramsIn.tau(paramsIn.tau <= tNow));
P = extractCostates(schemeData.grid, paramsIn.BRS(colons{:}, ii));
schemeData.uIn = schemeData.dynSys.optCtrl(tNow, schemeData.grid.xs, P, 'min');

end