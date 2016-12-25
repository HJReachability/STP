function rename(obj, new_name)

%% Change object folder
old_folder = obj.folder;
obj.folder = sprintf('%s_%s', class(obj), new_name);

%% Change folder name
if ispc
  system(sprintf('rename %s %s', old_folder, obj.folder));
else
  system(sprintf('mv %s %s', old_folder, obj.folder));
end

%% Search for properties that contain the substring 'filename'
P = properties(obj);
indices = strfind(P, 'filename');

%% Go through properties with substring 'filename' and replace the folder
for i = 1:length(P)
  index = indices{i};
  
  if ~isempty(index) && ~isempty(obj.(P{i}))
    % Change folder name
    obj.(P{i}) = strrep(obj.(P{i}), old_folder, obj.folder);
  end
end


end