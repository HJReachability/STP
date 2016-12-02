function RTTRS2D = migrateRTTRS(RTTRS, target_g2D, R_augment)

% Project RTTRS into 2D
[RTTRS_g2D, RTTRS2D_raw] = proj(RTTRS.g, -RTTRS.data, [0 0 1]);

% Create slightly bigger grid to augment the RTTRS
small_g2D_min = RTTRS_g2D.min - R_augment;
small_g2D_max = RTTRS_g2D.max + R_augment;
small_g2D_N = ceil((small_g2D_max - small_g2D_min) ./ ...
  (RTTRS_g2D.max - RTTRS_g2D.min) .* RTTRS_g2D.N);
small_g2D = createGrid(small_g2D_min, small_g2D_max, small_g2D_N);

% Migrate RTTRS set
RTTRS2D_raw = migrateGrid(RTTRS_g2D, RTTRS2D_raw, small_g2D);
RTTRS2D_raw = addCRadius(small_g2D, RTTRS2D_raw, R_augment);
RTTRS2D = migrateGrid(small_g2D, RTTRS2D_raw, target_g2D);

end