function vehicle = simulate_trajectory(g, vehicle);

current_time = 0;
t_end = vehicle.t_start;
t_step = vehicle.t_step;
index = 1;
reach_index = int64((vehicle.t_end - vehicle.t_start)/t_step);

f = figure;
while(current_time < t_end)
    
    % Process the input
    x_current = vehicle.x_nom(:,index);
    P = extractCostates(g,vehicle.reach(:,:,:,reach_index+index));
    p = calculateCostate(g,P,x_current);
    u = - 1*vehicle.turnRate_nom*sign(p(3));
    vehicle.u_nom(:,index) = u;
    
    % Let's do some plotting to ensure everything looks okay
    figure(f),
    hold on,
    plot(x_current(1), x_current(2), 'marker', 'o','markersize',5);
    [g2D, data2D] = proj2D(g, vehicle.reach(:,:,:,reach_index+index), [0 0 1], vehicle.x(3,1));
    [~,h2] = contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', 'm', 'linestyle','-' );
    drawnow;
    axis([-1 1 -1 1]);
    hold off;
    
    
    % Let's see if the current state is in the target set already
    location{1} = x_current(1);
    location{2} = x_current(2);
    location{3} = x_current(3);
    [location_index, ~] = getCellIndexes(g, location);
    [index1, index2, index3] = ind2sub(g.shape, location_index);
    if (vehicle.reach(index1, index2, index3, end) <= 0)
        vehicle.x_nom(:,index+1) = x_current;
        vehicle.obs_stoptime = current_time;
        delete(f);
        return;
    else
        % Actual next state
        x_next(1) = x_current(1) +  ...
            vehicle.v_nom*cos(x_current(3))*t_step;
        x_next(2) = x_current(2) + ...
            vehicle.v_nom*sin(x_current(3))*t_step;
        x_next(3) = x_current(3) + u*t_step;
    end
    
    vehicle.x_nom(:,index+1) = x_next;
    
    current_time = current_time + t_step;
    index = index+1;

    % delete function handle
    delete(h2);
end
vehicle.obs_stoptime = vehicle.t_start;
delete(f);