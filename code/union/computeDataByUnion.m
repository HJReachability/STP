function data = computeDataByUnion(base_g, base_data, g, target, pdims, adim)

% Default position dimensions
if nargin < 6
  pdims = [1 2];
end

% Default angle dimension
if nargin < 6
  adim = 3;
end

% Transfer base data to the same grid as the target set
base_data = migrateGrid(base_g, base_data, g);

%% Get indices of points inside target
in_target = find(target<0);

shifts_x = g.xs{pdims(1)}(in_target);
shifts_y = g.xs{pdims(2)}(in_target);
shifts = [shifts_x shifts_y];

if ~isempty(adim)
  thetas = g.xs{adim}(in_target);
end

data = inf(g.shape);
for i = 1:length(in_target)
  data_rot = rotateData(g, base_data, thetas(i), pdims, adim);
  data_rot_shift = shiftData(g, data_rot, shifts(i,:), pdims);
  data = min(data, data_rot_shift);
end

end
