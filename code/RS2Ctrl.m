function U = RS2Ctrl(RS, uMin, uMax)
% U = RS2Ctrl(RS, uMin, uMax)

U = zeros(size(RS.data));

for i = 1:length(RS.tau)
  switch RS.g.dim
    case 3
      P = extractCostates(RS.g, RS.data(:,:,:,i));
      U(:,:,:,i) = (P{3} >= 0) * uMin + (P{3} < 0) * uMax;
      
    otherwise
      error('This function has only been implemented for 3D systems!')
  end
end

end