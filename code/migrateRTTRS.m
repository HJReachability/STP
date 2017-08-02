function [g2D, RTTRS2D] = migrateRTTRS(RTTRS, R_augment)

% Project RTTRS into 2D
[RTTRS_g2D, RTTRS2D] = proj(RTTRS.g, -RTTRS.data, [0 0 1]);   

% Create slightly bigger grid to augment the RTTRS
g2D_min = RTTRS_g2D.min - 2*R_augment;
g2D_max = RTTRS_g2D.max + 2*R_augment;
g2D_N = ceil((g2D_max - g2D_min) ./ ...
  (RTTRS_g2D.max - RTTRS_g2D.min) .* RTTRS_g2D.N);
g2D = createGrid(g2D_min, g2D_max, g2D_N);

% Migrate RTTRS set
RTTRS2D = migrateGrid(RTTRS_g2D, RTTRS2D, g2D);
RTTRS2D = addCRadius(g2D, RTTRS2D, R_augment);   %%%%%%%%%%%%%%%%%%%

end