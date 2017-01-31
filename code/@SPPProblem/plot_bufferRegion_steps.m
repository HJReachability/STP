function plot_bufferRegion_steps(obj)

% Plot base obstacle

f = figure;
f.Color = 'white';
f.Position = [100 100 800 600];

for i = 1:4
  subplot(2,2,i)
  
  switch i
    case 1
      load(obj.RTTRS_filename)
      visSetIm(RTTRS.g, RTTRS.data);
      title('Base Obstacle ($\mathcal{M}(\underline{t})$)', ...
        'Interpreter', 'LaTeX', 'FontSize', 16)
      
    case 2
      load(obj.CARS_filename)
      visSetIm(CARS.g, CARS.data(:,:,:,end));
      title('Avoid Region ($\mathcal{V}^A(0, t^\mathrm{IAT})$)', ...
        'Interpreter', 'LaTeX', 'FontSize', 16)
      zlim([0 2*pi])
      set(gca, 'ztick', [0 pi 2*pi])
      set(gca, 'zticklabels', {'0', '\pi', '2\pi'})
      
    case 3
      RTTRSdata = migrateGrid(RTTRS.g, -RTTRS.data, CARS.g);
      sepRegion = computeDataByUnion(CARS.g, CARS.data(:,:,:,end), ...
        CARS.g, RTTRSdata, [1 2], 3, false);
      visSetIm(CARS.g, sepRegion);
      title('Separation Region ($\mathcal{S}(\underline{t})$)', ...
        'Interpreter', 'LaTeX', 'FontSize', 16)
      
      xlabel('x (m)')
      ylabel('y (m)')
      zlabel('\theta (rad)')
      
      zlim([0 2*pi])
      set(gca, 'ztick', [0 pi 2*pi])
      set(gca, 'zticklabels', {'0', '\pi', '2\pi'})      
      
    case 4
      load(obj.minMinBRS_filename)
      
      rdi = obj.remaining_duration_ind;
      CARSdata = migrateGrid(CARS.g, CARS.data(:,:,:,rdi), minMinBRS.g);
      
      bdi = obj.buffer_duration_ind;
      RBR = computeDataByUnion(minMinBRS.g, minMinBRS.data(:,:,:,bdi), ...
        minMinBRS.g, CARSdata, [1 2], 3, false);
      
      visSetIm(minMinBRS.g, RBR);
      
      title({'(Relative) Buffer Region', ...
        '\qquad($\mathcal{V}^B(0, t^\mathrm{BRD})$)'}, 'Interpreter', ...
        'LaTeX', 'FontSize', 16)
      
      zlim([-pi pi])
      set(gca, 'ztick', [-pi 0 pi])
      set(gca, 'zticklabels', {'-\pi', '0', '\pi'})      
  end
  
  box on
  grid on
  axis square
  
end



end