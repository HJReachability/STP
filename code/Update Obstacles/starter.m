%% Function starts here

clear all;
close all;
addpath ./obs_vehicle1

%---------------------------------------------------------------------------
% Grid of joint space
Nx = 51;
N = [ Nx; Nx; Nx];
grid_min = [ -1; -1; 0];
grid_max = [ +1; +1; 2*pi];
pdDims = [3];
% Create the computation grid.
g = createGrid(grid_min, grid_max, N, pdDims);

%---------------------------------------------------------------------------
% Time parameters
t_start = 0;
t_end = 4;
t_step = 0.01;
steps = int64((t_end - t_start)/t_step);

%---------------------------------------------------------------------------
% Create default vehicle object

% Integration parameters
vehicle.reach_accuracy = 'low';
vehicle.t_step = t_step; % can also be different for obstacle simulation
vehicle.t_start = t_end; % Initialize to the end time
vehicle.t_end = t_end;
vehicle.capture_radius = 0.7*0.1;

% Other vehicle parameters
vehicle.v_nom = 0.75;
vehicle.velocity = 0.25;
vehicle.turnRate = 1;

% Input uncertainty models: box
vehicle.disrurbance_type = 'box';
percent = 0.1; % (% of disturbance)
vehicle.disturbance_mag = percent*[(vehicle.velocity + vehicle.v_nom), (vehicle.velocity + vehicle.v_nom), 2*vehicle.turnRate]'; %assuming symmetric lower and upper bounds;

% Define and initialize reachable set matrix
vehicle.reach = 1e6*ones(g.shape);

% Time to Reach Values
vehicle.init_TTR = 1e6;
vehicle.final_TTR = 1e6;

% Load the starting position of the vehicle
vehicle.x_init = [  0.5, 0, pi]';



% Target sets of vehicles
% Target position and target radius matrix should be provided
target_pos = zeros(vnum,2); % Number of vehicles x Number of states (to be reached)
target_pos(1,:) = [0.7,0.2];
target_pos(2,:) = [-0.7,0.2];
target_pos(3,:) = [0.7,-0.7];
target_pos(4,:) = [-0.7,-0.7];

target_radius = zeros(vnum,1);
target_radius(1,1) = 0.1;
target_radius(2,1) = 0.1;
target_radius(3,1) = 0.1;
target_radius(4,1) = 0.1;

for i=1:vnum
    allVehicles{i}.reach(:,:,:,steps+1) = sqrt((g.xs{1} - target_pos(i,1)).^2 +...
        (g.xs{2} - target_pos(i,2)).^2) - target_radius(i,1);
end

% Plot the target sets of the vehicles
for i=1:vnum
    figure(allVehicles{i}.mast_fig);
    if(~isempty(allVehicles{i}.fig_hand))
        subplot(allVehicles{i}.fig_hand);
    end
    hold on,
    [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1]);
    contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
    drawnow;
    axis equal;
    hold off,
end

% Times to plot reachable set and collision set for each vehicle
for i=1:vnum
    allVehicles{i}.tplot = 1.2;
end


% ---------------------------------------------------------------------------
%% Compute the obstacle and reachable set shapes

for i=1:vnum
    allVehicles{i} = ComputeReachSet(g, allVehicles{i}, allVehicles(1:i-1),'stop');
    if (i ~= vnum) % Do not update collision obstacles for the last vehicle
        allVehicles{i} = ComputeCollisionObs(g, allVehicles{i}, 'stop');
        % Fix collision matrix so that actual obstacles are towards the end
        temp0 = 1e6*ones(g.shape);
        temp0  = repmat(temp0,  [ones(1,g.dim),steps+1]);
        % Extract the relevant collision matrix
        num_steps = int64(allVehicles{i}.t_start/t_step +1);
        start_index = int64((t_end-allVehicles{i}.t_start)/t_step);
        temp0(:,:,:,start_index+1:end) = allVehicles{i}.collisionmat(:,:,:,1:num_steps);
        allVehicles{i}.collisionmat = temp0;
        clear('temp0');
    end
    t_start = max(t_start, allVehicles{i}.t_start);
    save('ex1', '-v7.3');
    pause;
end

t_end = t_start;
t_start = 0;
steps = int64((t_end - t_start)/t_step);

% Adjust the matrix sizes
for i=1:vnum
   % First change the end time
   allVehicles{i}.t_end = t_end;
   
   % Adjust the start time
   allVehicles{i}.t_start = t_end - allVehicles{i}.t_start;
   
   % Adjust the collision matrix
   % Remove the collision matrix part that is not required
   temp1 = 1e6*ones(g.shape);
   temp1  = repmat(temp1,  [ones(1,g.dim),steps+1]);
   % Extract the relevant collision matrix
   num_steps = int64((allVehicles{i}.t_end - allVehicles{i}.t_start)/t_step +1);
   start_index = int64(allVehicles{i}.t_start/t_step);
   temp1(:,:,:,start_index+1:end) = allVehicles{i}.collisionmat(:,:,:,end-num_steps+1:end);
   allVehicles{i}.collisionmat = temp1;
   clear('temp1');
%    temp1(:,:,:,start_index+1:end) = allVehicles{i}.collisionmat(:,:,:,1:num_steps);
%   allVehicles{i}.collisionmat(:,:,:,1:end-num_steps) = [];
   
   % Adjust the reach matrix: simply delete the extra rows
   temp2 = 1e6*ones(g.shape);
   temp2  = repmat(temp2,  [ones(1,g.dim),steps+1]);
   % Extract the relevant reach matrix
   temp2(:,:,:,start_index+1:end) = allVehicles{i}.reach(:,:,:,end-num_steps+1:end);
   allVehicles{i}.reach = temp2;
   clear('temp2');
   
   % Adjust the optU matrix
   temp3 = zeros(g.shape);
   temp3  = repmat(temp3,  [ones(1,g.dim),steps+1]);
   % Extract the relevant optU matrix
   temp3(:,:,:,start_index+1:end) = allVehicles{i}.optU(:,:,:,end-num_steps+1:end);
   allVehicles{i}.optU = temp3;
   clear('temp3');
   
   % Actual input trajectory
   allVehicles{i}.u = zeros(1,steps);

   % Applied disturbance
   allVehicles{i}.d = zeros(3,steps);

   % state trajectory (initialized to the initial state)
   temp_x = allVehicles{i}.x(:,1);
   allVehicles{i}.x = zeros(3,steps+1);
   allVehicles{i}.x(:,1:start_index+1) = repmat(temp_x, 1, start_index+1);
   
end    

% Process inputs for the vehicles starting at t=0 
current_time = t_start;
end_time = min(t_end, current_time + t_step);
index = 1;

while(current_time < t_end)
    
    for i=1:vnum
        if(~allVehicles{i}.reach_flag && current_time >= allVehicles{i}.t_start)
            
            % Process the input
            x_current = allVehicles{i}.x(:,index);
            P = extractCostates(g,allVehicles{i}.reach(:,:,:,index));
            p = calculateCostate(g,P,x_current);
            u = - 1*allVehicles{i}.turnRate*sign(p(3));
            v = -1*allVehicles{i}.velocity*(p(1)*cos(x_current(3)) + p(2)*sin(x_current(3)));
            allVehicles{i}.u(:,index) = u;
            
            % Applied disturbance
            allVehicles{i}.d(:,index) = applyDisturbance(allVehicles{i}, p, 'worst');
            d1 = allVehicles{i}.d(1,index);
            d2 = allVehicles{i}.d(2,index);
            d3 = allVehicles{i}.d(3,index);
            
            % Let's see if the current state is in the target set already
            location{1} = x_current(1);
            location{2} = x_current(2);
            location{3} = x_current(3);
            [location_index, ~] = getCellIndexes(g, location);
            [index1, index2, index3] = ind2sub(g.shape, location_index);
            if (allVehicles{i}.reach(index1, index2, index3, end) <= 0)
                x_next = x_current;
                allVehicles{i}.final_TTR = min(allVehicles{i}.final_TTR, (index-1)*t_step);
                allVehicles{i}.reach_flag = 1;
            else
                % Actual next state
                x_next(1) = x_current(1) + d1*t_step + ...
                    (allVehicles{i}.v_nom+v)*cos(x_current(3))*t_step;
                x_next(2) = x_current(2) + d2*t_step + ...
                    (allVehicles{i}.v_nom+v)*sin(x_current(3))*t_step;
                x_next(3) = x_current(3) + d3*t_step + ...
                    u*t_step;
            end
        else
            x_next = allVehicles{i}.x(:,index);
        end
        allVehicles{i}.x(:,index+1) = x_next;
        
        % Process the trajectory
        figure(allVehicles{i}.mast_fig);
        if(~isempty(allVehicles{i}.fig_hand))
            subplot(allVehicles{i}.fig_hand);
        end
        hold on,
        plot(x_next(1), x_next(2), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
        delete(allVehicles{i}.quiver_hand);
        dirn = 0.2*[cos(x_next(3)) sin(x_next(3))];
        allVehicles{i}.quiver_hand = quiver(x_next(1),x_next(2), dirn(1), dirn(2), ...
            'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
        drawnow;
        axis equal;
        hold off,
    end
    
    current_time = current_time + t_step;
    end_time = min(t_end, current_time + t_step);
    index = index+1
end

% Plot the trajectories
f2 = figure; % Figure with all the trajectories
figure(f2);
hold on,
for i=1:vnum
    % Plot the target sets of the vehicles
    [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1]);
    contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
    % Plot the trajectories of the vehicles
    plot(allVehicles{i}.x(1,:), allVehicles{i}.x(2,:), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
    axis equal;
    drawnow;
end
hold off;


% Plot the initial positions of the vehicles
for i=1:vnum
    figure(allVehicles{i}.mast_fig);
    if(~isempty(allVehicles{i}.fig_hand))
        subplot(allVehicles{i}.fig_hand);
    end
    plot(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
    hold on,
    dirn = 0.2*[cos(allVehicles{i}.x(3,1)) sin(allVehicles{i}.x(3,1))];
    allVehicles{i}.quiver_hand = quiver(allVehicles{i}.x(1,1), allVehicles{i}.x(2,1), dirn(1), dirn(2), ...
        'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
    drawnow;
    axis equal;
    hold off,
end