i = 1; % Vehicle number

obstacles = cell(1, 339);

% Index range (205 to 337) for Obstacles induced by Vehicle 1
for index = 205:337
   
% Get the 2D Projection of the obstacle
data = allVehicles{i}.collisionmat(:,:,:,index);
[g2D, data2D] = proj2D(g, data, [0 0 1]);

% % Draw the 2D-contour
% f = figure;
% figure(f),
% [~,h1] = contour(g2D.xs{1}, g2D.xs{2}, data2D, [0 0], 'color', 'r', 'Linestyle', '-');
% drawnow;

% % Take the 1D Projection on x and y axes
% data1D_x = min(data2D,[],2);
% data1D_y = min(data2D,[],1);
% 
% % Co-ordinates of rectangle around the shape
% xmin_index = min(find(data1D_x <= 0));
% xmax_index = max(find(data1D_x <= 0));
% ymin_index = min(find(data1D_y <= 0));
% ymax_index = max(find(data1D_y <= 0));
% 
% xmin = g2D.xs{1}(xmin_index,1);
% xmax = g2D.xs{1}(xmax_index,1);
% ymin = g2D.xs{2}(1,ymin_index);
% ymax = g2D.xs{2}(1,ymax_index);
% 
% % Plot the square around the min and max values
% figure(f),
% hold on;
% plot([xmin xmin] , [ymin ymax], 'color', 'b', 'Linestyle', '--');
% plot([xmax xmax] , [ymin ymax], 'color', 'b', 'Linestyle', '--');
% plot([xmin xmax] , [ymin ymin], 'color', 'b', 'Linestyle', '--');
% plot([xmin xmax] , [ymax ymax], 'color', 'b', 'Linestyle', '--');

% % Get the major axis direction
% dirn = -atan((ymax - ymin)/(xmax - xmin));
% b =  norm([xmax - xmin, ymax - ymin])/2; % Major axis
% a =  0.2; % Minor axis
% cenx = (xmin + xmax)/2;
% ceny = (ymin + ymax)/2;
% 
% ellipse = sqrt(((g2D.xs{1}-cenx)*cos(dirn) + (g2D.xs{2}-ceny)*sin(dirn)).^2 /a^2 + ((g2D.xs{1}-cenx)*sin(dirn) - (g2D.xs{2}-ceny)*cos(dirn)).^2 /b^2) -1;
% figure(f),
% hold on;
% [~,h2] = contour(g2D.xs{1}, g2D.xs{2}, ellipse, [0 0], 'color', 'k', 'Linestyle', '-.');
% drawnow;
% hold off;

% Use the regionprops to fit an ellipse
out = regionprops(data2D<=0, 'Centroid','MinorAxisLength','MajorAxisLength', 'Orientation');
out = out(1);

% Get the major axis direction
dirn = (pi/180)*out.Orientation;
b =  g2D.dx(2)*out.MajorAxisLength/2; % Major axis
a =  g2D.dx(1)*out.MinorAxisLength/2;; % Minor axis
cenx = -1+g2D.dx(1)*out.Centroid(2);
ceny = -1+g2D.dx(2)*out.Centroid(1);

% ellipse = sqrt(((g2D.xs{1}-cenx)*cos(dirn) + (g2D.xs{2}-ceny)*sin(dirn)).^2 /a^2 + ((g2D.xs{1}-cenx)*sin(dirn) - (g2D.xs{2}-ceny)*cos(dirn)).^2 /b^2) -1;
% figure(f),
% hold on;
% [~,h2] = contour(g2D.xs{1}, g2D.xs{2}, ellipse, [0 0], 'color', 'k', 'Linestyle', '-.');
% drawnow;
% hold off;

obstacles{index}.cenx = cenx;
obstacles{index}.ceny = ceny;
obstacles{index}.dirn = dirn;
obstacles{index}.a = a;
obstacles{index}.b = b;

end

save('obs_vehicle1', 'obstacles');