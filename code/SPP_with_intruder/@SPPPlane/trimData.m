function trimData(obj, fieldsToRemove)
% trimData(obj, fieldsToRemove)
%     Trims large data files so that a smaller file is saved to run simulations

for i = 1:length(fieldsToRemove)
  obj.(fieldsToRemove{i}) = [];
end

end