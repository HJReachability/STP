function Obs = FRS2Obs(FRS)

Obs2D = zeros(FRS.g.N(1), FRS.g.N(2), length(FRS.tau));

[g, Obs2D(:,:,1)] = proj2D(FRS.g, FRS.data(:,:,:,1), [0 0 1], 'min');

for i = 2:length(FRS.tau)
  [~, Obs2D(:,:,i)] = proj2D(FRS.g, FRS.data(:,:,:,i), [0 0 1], 'min');
end

Obs.g = g;
Obs.O2D = Obs2D;
Obs.tau = FRS.tau;

end