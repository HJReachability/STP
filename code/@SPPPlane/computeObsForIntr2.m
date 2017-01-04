function computeObsForIntr2(obj, SPPP, bufferRegion, FRSBRS, veh, save_png)
% computeObsForIntr2(obj, g, CARS, rawAugObs)
%     Computes the FRS + BRS + IES obstacles assuming the RTT method
%     for SPP with intruders method 2

if nargin < 6
  save_png = true;
end

obj.obsForIntr_tau = obj.nomTraj_tau;
obj.obsForIntr = inf([SPPP.g.N' length(obj.nomTraj_tau)]);
obj.obs2D_tau = obj.nomTraj_tau;
obj.obs2D = zeros([SPPP.g2D.N' length(obj.nomTraj_tau)]);

len_tIAT = length(FRSBRS.FRS.tau);

if save_png
  if ispc
    folder = sprintf('%s\\%s\\%d', SPPP.folder, mfilename, veh);
    system(sprintf('mkdir %s', folder));
  else
    folder = sprintf('%s/%s/%d', SPPP.folder, mfilename, veh);
    system(sprintf('mkdir -p %s', folder));
  end
  
  figure
  plot(obj.nomTraj(1,:), obj.nomTraj(2,:), 'k.-')
  hold on
end

for i = 1:length(obj.nomTraj_tau)
  fprintf('  Augmenting obstacle %d of %d\n', i, length(obj.nomTraj_tau))
  
  % Determine trajectory and obstacle indices for each of the raw obstacles
  trajIndBuffer = i;
  trajIndFRS = max(1, i-len_tIAT+1);
  trajIndBRS = min(length(obj.nomTraj_tau), i+len_tIAT-1);
  
  % Buffer region always has the same obstacle index
  obsIndFRS = min(i, len_tIAT);
  obsIndBRS = min(length(obj.nomTraj_tau)-i+1, len_tIAT);
  
  %% Add buffer region
  pBuffer = obj.nomTraj(1:2, trajIndBuffer);
  tBuffer = obj.nomTraj(3, trajIndBuffer);
  obsBufferi_rotated = rotateData(bufferRegion.g, bufferRegion.data, ...
    tBuffer, [1 2], 3);
  obsBufferi_gShifted = shiftGrid(bufferRegion.g, [pBuffer; 0]);
  obsBufferi = migrateGrid(obsBufferi_gShifted, obsBufferi_rotated, SPPP.g);
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), obsBufferi);
  
  %% Add FRS obstacle
  pFRS = obj.nomTraj(1:2, trajIndFRS);
  tFRS = obj.nomTraj(3, trajIndFRS);
  obsFRSi_rotated = rotateData(FRSBRS.g, ...
    FRSBRS.FRS.data(:,:,:,obsIndFRS), tFRS, [1 2], 3);
  obsFRSi_gShifted = shiftGrid(FRSBRS.g, [pFRS; 0]);
  obsFRSi = migrateGrid(obsFRSi_gShifted, obsFRSi_rotated, SPPP.g);
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), obsFRSi);
  
  %% Project to 2D
  [~, obj.obs2D(:,:,i)] = proj(SPPP.g, obj.obsForIntr(:,:,:,i), [0 0 1]);
  
  if i == 1
    h2D = visSetIm(SPPP.g2D, obj.obs2D(:,:,i), 'b');
  else
    h2D.ZData = obj.obs2D(:,:,i);
  end
  
  %% Add BRS obstacle
  pBRS = obj.nomTraj(1:2, trajIndBRS);
  tBRS = obj.nomTraj(3, trajIndBRS);
  obsBRSi_rotated = rotateData(FRSBRS.g, ...
    FRSBRS.BRS.data{1}(:,:,:,obsIndBRS), tBRS, [1 2], 3);
  obsBRSi_gShifted = shiftGrid(FRSBRS.g, [pBRS; 0]);
  obsBRSi = migrateGrid(obsBRSi_gShifted, obsBRSi_rotated, SPPP.g);
  
  obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), obsBRSi);
  
  %% Add case 4 (intruder affects this vehicle first, then lower priority)
  for j = 1:SPPP.remaining_duration_ind
    base_obs_ind = max(1, i-len_tIAT+j);
    pFRSBRS = obj.nomTraj(1:2, base_obs_ind);
    tFRSBRS = obj.nomTraj(3, base_obs_ind);
    
    FRS_ind = min(base_obs_ind, len_tIAT);
    BRS_ind = j; % 1 to remaining_duration
    
    obsFRSBRSi_rotated = rotateData(FRSBRS.g, ...
      FRSBRS.BRS.data{FRS_ind}(:,:,:,BRS_ind), tFRSBRS);
    obsFRSBRSi_gShifted = shiftGrid(FRSBRS.g, [pFRSBRS; 0]);
    obsFRSBRSi = migrateGrid(obsFRSBRSi_gShifted, obsFRSBRSi_rotated, SPPP.g);
    
    obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), obsFRSBRSi);
  end
  
  %% Add case 5 (intruder affects lower priority first, then this vehicle)
  for j = SPPP.remaining_duration_ind+1:len_tIAT
    base_obs_ind = max(1, i + j - SPPP.remaining_duration_ind);
    
    if base_obs_ind > length(obj.nomTraj_tau)
      break
    end
    
    pFRSBRS = obj.nomTraj(1:2, base_obs_ind);
    tFRSBRS = obj.nomTraj(3, base_obs_ind);
    
    FRS_ind = min(base_obs_ind, SPPP.remaining_duration_ind);
    BRS_ind = j; 
    
    obsFRSBRSi_rotated = rotateData(FRSBRS.g, ...
      FRSBRS.BRS.data{FRS_ind}(:,:,:,BRS_ind), tFRSBRS);
    obsFRSBRSi_gShifted = shiftGrid(FRSBRS.g, [pFRSBRS; 0]);
    obsFRSBRSi = migrateGrid(obsFRSBRSi_gShifted, obsFRSBRSi_rotated, SPPP.g);
    
    obj.obsForIntr(:,:,:,i) = min(obj.obsForIntr(:,:,:,i), obsFRSBRSi);
  end
  
  %% Exclude target set
  obj.obsForIntr(:,:,:,i) = max(obj.obsForIntr(:,:,:,i), -obj.target);
  
  %% Project to 2D
  [~, obj.obs2D(:,:,i)] = proj(SPPP.g, obj.obsForIntr(:,:,:,i), [0 0 1]);
  
  if i == 1
    h2D2 = visSetIm(SPPP.g2D, obj.obs2D(:,:,i), 'k');
  else
    h2D2.ZData = obj.obs2D(:,:,i);
  end
  drawnow
  export_fig(sprintf('%s/%d', folder, i), '-png', '-m2')  
end
end