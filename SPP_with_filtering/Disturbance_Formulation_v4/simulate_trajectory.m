function x_sim = simulate_trajectory(g, vehicle, t_end)
t_step = vehicle.t_step;
steps = t_end/t_step;
xinit = vehicle.x(:,1);
x_current = xinit;
x_sim(:,1) = xinit;
figure,
for index=1:steps
    % Process the input
    P = extractCostates(g,vehicle.cons_reach(:,:,:,index));
    p = calculateCostate(g,P,x_current);
    u = - 1*vehicle.turnRate*sign(p(3));
    
    % Applied disturbance
    d = applyDisturbance(vehicle, p, 'worst');
    
    % Let's see if the current state is in the target set already
    location{1} = x_current(1);
    location{2} = x_current(2);
    location{3} = x_current(3);
    [location_index, ~] = getCellIndexes(g, location);
    [index1, index2, index3] = ind2sub(g.shape, location_index);
    if (vehicle.cons_reach(index1, index2, index3, end) > 0)
        % Actual next state
        x_next(1) = x_current(1) + d(1)*t_step + ...
            vehicle.velocity*cos(x_current(3))*t_step;
        x_next(2) = x_current(2) + d(2)*t_step + ...
            vehicle.velocity*sin(x_current(3))*t_step;
        x_next(3) = x_current(3) + d(3)*t_step + ...
            u*t_step;
    else
        index
        x_sim(:,index+1) = x_next;
        return;
        
    end
    x_current = x_next;
    x_sim(:,index+1) = x_current;
    
    % Plot functions
    [g2D, data2D] = proj2D(g, vehicle.cons_reach(:,:,:,index), [0 0 1], x_current(3));
    [~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'b', 'linestyle', '--');
    hold on;
    plot(x_next(1), x_next(2), 'marker', 'o', 'color', vehicle.fig_color,'markersize',5);
    delete(vehicle.quiver_hand);
    dirn = 0.2*[cos(x_next(3)) sin(x_next(3))];
    vehicle.quiver_hand = quiver(x_next(1),x_next(2), dirn(1), dirn(2), ...
            'maxheadsize', 50, 'marker', '*', 'color', vehicle.fig_color);
    drawnow;
    delete(h2);
end