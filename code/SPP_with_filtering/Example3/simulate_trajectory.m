function vehicle = simulate_trajectory(g, vehicle);

current_time = 0;
t_end = vehicle.t_end;
t_step = vehicle.t_step;
index = 1;
reach_index = int64(vehicle.t_start/t_step);
vehicle.x_nom(:,reach_index+1) = vehicle.x(:,1);

while(current_time < t_end)
    if (current_time >= vehicle.t_start)
        % Process the input
        x_current = vehicle.x_nom(:,index);
        P = extractCostates(g,vehicle.reach(:,:,:,index));
        p = calculateCostate(g,P,x_current);
        u = - 1*vehicle.turnRate_nom*sign(p(3));
        vehicle.u_nom(:,index) = u;
        
        % Let's see if the current state is in the target set already
        location{1} = x_current(1);
        location{2} = x_current(2);
        location{3} = x_current(3);
        [location_index, ~] = getCellIndexes(g, location);
        [index1, index2, index3] = ind2sub(g.shape, location_index);
        if (vehicle.reach(index1, index2, index3, end) <= 0)
            vehicle.x_nom(:,index+1:end) = repmat(x_current, 1, size(vehicle.x_nom,2)-index);
            return;
        else
            % Actual next state
            x_next(1) = x_current(1) +  ...
                vehicle.v_nom*cos(x_current(3))*t_step;
            x_next(2) = x_current(2) + ...
                vehicle.v_nom*sin(x_current(3))*t_step;
            x_next(3) = x_current(3) + u*t_step;
        end
    else
        x_next = vehicle.x_nom(:,index);
    end
    vehicle.x_nom(:,index+1) = x_next;
    
    current_time = current_time + t_step;
    index = index+1
end