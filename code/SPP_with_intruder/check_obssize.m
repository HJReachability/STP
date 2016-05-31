function [data, update_flag] = check_obssize(g, data, reset_radius)

% Compute center and radius of all islands
[~, rNs, rs, cs] = findIslands(g, data, 0);

% Calculate the number of islands
num_isl = length(cs);

% Initialize the update flag
updat_flag = false;

% Now add the obstacle corresponding to each islands
for i=1:num_isl
    if(min(rNs{i}) <= 1)
        reset_indices = find(rNs{i} <= 1);
        rs{i}(reset_indices) = reset_radius(reset_indices)';
        collisionObs1 = sqrt((1/rs{i}(1)^2)*(g.xs{1} - cs{i}(1)).^2 + (1/rs{i}(2)^2)*(g.xs{2} - cs{i}(2)).^2 +...
            (1/rs{i}(3)^2)*(g.xs{3} - cs{i}(3)).^2) - 1;
        collisionObs2 = sqrt((1/rs{i}(1)^2)*(g.xs{1} - cs{i}(1)).^2 + (1/rs{i}(2)^2)*(g.xs{2} - cs{i}(2)).^2 +...
            (1/rs{i}(3)^2)*(-2*pi + g.xs{3} - cs{i}(3)).^2) - 1;
        collisionobs = min(collisionObs1, collisionObs2);
        data = min(data, collisionobs);
        disp('Obstacle updated!');
        update_flag = true;
    end
end







%% Deprecated code
% for i=1:3
% [xhi(i), xlo(i), c(i), gpts(i), r(i)] = findCandR(g, data, i);
% end
% 
% % We will store all possible small obstacle centers in the cell cen_obs
% % Assuming that the size of multiple divisions of obstacles will roughly be
% % same
% 
% for i=1:3
%     if(gpts(i) <= 2)
%         if(round(xhi(i)-xlo(i)/g.dx(i)) > 1 && round(xhi(i)-2*pi-xlo(i)/g.dx(i)) > 1) % 2*pi condition is for turn rate
%             cen_obs{i} = [xhi(i), xlo(i)];
%             r(i) = vehicle.state_uncertainty_axis(i);
%         else
%             cen_obs{i} = [c(i)];
%             r(i) = vehicle.state_uncertainty_axis(i);
%         end
%     else
%         cen_obs{i} = [c(i)];
%     end
% end
% 
% % Now create all possible obstacles
% a = len(cen_obs{1});
% b = len(cen_obs{2});
% c = len(cen_obs{3});
% 
% for i=1:a
%     for j=1:b
%         for k=1:c
%             collisionObs1 = sqrt((1/r(1)^2)*(g.xs{1} - c(i)).^2 + (1/r2^2)*(g.xs{2} - c2).^2 +...
%                 (1/r3^2)*(g.xs{3} - c3).^2) - 1;
%             collisionObs2 = sqrt((1/r1^2)*(g.xs{1} - c1).^2 + (1/r2^2)*(g.xs{2} - c2).^2 +...
%                 (1/r3^2)*(-2*pi + g.xs{3} - c3).^2) - 1;
%             data = min(collisionObs1, collisionObs2);
%             disp('Obstacle updated!');
% 
% 
% % If need be adjust the cen
% r1 = max(vehicle.state_uncertainty_axis(1),r1);
% r2 = max(vehicle.state_uncertainty_axis(2),r2);
% r3 = max(vehicle.state_uncertainty_axis(3),r3);
% 
% 
% if(gpts1<=2 || gpts2<=2 || gpts3 <= 2)
%     collisionObs1 = sqrt((1/r1^2)*(g.xs{1} - c1).^2 + (1/r2^2)*(g.xs{2} - c2).^2 +...
%         (1/r3^2)*(g.xs{3} - c3).^2) - 1;
%     collisionObs2 = sqrt((1/r1^2)*(g.xs{1} - c1).^2 + (1/r2^2)*(g.xs{2} - c2).^2 +...
%         (1/r3^2)*(-2*pi + g.xs{3} - c3).^2) - 1;
%     data = min(collisionObs1, collisionObs2);
%     disp('Obstacle updated!');
% end
% 
%         
%     
%     function [xhi, xlo, center, gpts, r] = findCandR(g, data, dim)
%     
%     % First let's find the 1D projections
%     if(dim==1)
%         [~, data2D] = proj2D(g, data, [0 0 1]);
%         data1D = squeeze(min(data2D, [], 2));
%     end
%     
%     if(dim==2)
%         [~, data2D] = proj2D(g, data, [0 0 1]);
%         data1D = squeeze(min(data2D, [], 1));
%     end
%     
%     if(dim==3)
%         [~, data2D] = proj2D(g, data, [0 1 0]);
%         data1D = squeeze(min(data2D, [], 1));
%     end
%     
%     % Now compute the number of grid points
%     indices = find(data1D <= 0);
%     gpts = length(indices);
%     
%     % Also compute the center of the obstacle in this dimension
%     xs = g.xs{1}(indices,1,1);
%     xlo = min(xs);
%     xhi = max(xs);
%     center = (xhi + xlo)/2;
%     r = (xhi - xlo)/2;
%     
