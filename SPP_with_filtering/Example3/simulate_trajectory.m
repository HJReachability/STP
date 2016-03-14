function vehicle = simulate_trajectory(g, vehicle);

current_time = 0;
t_end = vehicle.t_start;
t_step = vehicle.t_step;
index = 1;
reach_index = int64((vehicle.t_end - vehicle.t_start)/t_step);

while(current_time < t_end)
    
    % Process the input
    x_current = vehicle.x_nom(:,index);
    P = extractCostates(g,vehicle.reach(:,:,:,reach_index+index));
    p = calculateCostate(g,P,x_current);
    u = - 1*vehicle.turnRate_nom*sign(p(3));
    vehicle.u_nom(:,index) = u;
    
    % Actual next state
    x_next(1) = x_current(1) +  ...
        vehicle.v_nom*cos(x_current(3))*t_step;
    x_next(2) = x_current(2) + ...
        vehicle.v_nom*sin(x_current(3))*t_step;
    x_next(3) = x_current(3) + u*t_step;
    
    vehicle.x_nom(:,index+1) = x_next;
    
    current_time = current_time + t_step;
    index = index+1;
end
