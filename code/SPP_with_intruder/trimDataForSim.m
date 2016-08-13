function Q = trimDataForSim(Q, fieldsToRemove)
% trimDataForSim(Q, fieldsToRemove)
%     Trims large data files so that a smaller file is saved to run simulations

for veh = 1:length(Q)
  for i = 1:length(fieldsToRemove)
    if isfield(Q{veh}.data, fieldsToRemove{i})
      Q{veh} = rmfield(Q{veh}.data, fieldsToRemove{i});
    end
  end
end
end