%% Function starts here

clear all;
close all;
addpath ~/Documents/MATLAB/helperOC
load RB_data

% f1 = figure; % Figure with reachable set and obstacles

%---------------------------------------------------------------------------
% Grid of joint space
Nx = 151;

% Create the computation grid.
g.dim = 3;
g.min = [ -1; -1; 0];
g.max = [ +1; +1; 2*pi];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; 73];
% Need to trim max bound in \psi (since the BC are periodic in this dimension).
g.max(3) = g.max(3) * (1 - 1 / g.N(3));
g = processGrid(g);

%---------------------------------------------------------------------------
% Time parameters
t_start = 0;
t_end = 6;
t_step = 0.01;
steps = int64((t_end - t_start)/t_step);

%---------------------------------------------------------------------------
% Create default vehicle object
% There should be one vehicle object for each vehicle

% Integration parameters
vehicle.reach_accuracy = 'low';
vehicle.t_step = t_step; % can also be different for obstacle simulation
vehicle.t_start = t_end; % Initialize to the end time
vehicle.t_end = t_end;
vehicle.capture_radius = 0.1;
vehicle.bubble_radius = 0.075;

% Other vehicle parameters
vehicle.v_nom = 0.75;
vehicle.velocity = 0.25;
vehicle.turnRate_nom = 0.6;
vehicle.turnRate = 0.4;

% Input uncertainty models: box
vehicle.disrurbance_type = 'box';
percent = 0.1; % (% of disturbance)
vehicle.disturbance_mag = [0.1, 0.1, 0.2]'; %assuming symmetric lower and upper bounds;

% Define and initialize reachable set matrix
vehicle.reach = 1e6*ones(g.shape);
vehicle.reach  = repmat(vehicle.reach,  [ones(1,g.dim),steps+1]);

% Actual input trajectory
vehicle.u = zeros(1,steps);
vehicle.v = zeros(1,steps);

% Applied disturbance
vehicle.d = zeros(3,steps);

% state trajectory (initialized to the initial state)
vehicle.x = zeros(3,steps+1);

% Store the nominal trajectory for each vehicle
vehicle.x_nom = zeros(3,steps+1);
vehicle.u_nom = zeros(3,steps+1);

% Reach and Obstacle handles
vehicle.reach_hand1 = {};
vehicle.reach_hand2 = {};
vehicle.obs_hand = {};
vehicle.fig_hand = {};
vehicle.plot_data = {};
vehicle.plot_grid = {};

% Time to Reach Values
vehicle.initial_TTR = 1e6;
vehicle.final_TTR = 1e6;
vehicle.reach_flag = 0;

%---------------------------------------------------------------------------
% Create an object for each vehicle
vnum = 4; % total number of vehicles
for i=1:vnum
    allVehicles{i} = vehicle;
end

% Change the accuracy for vehicle 3 so that it works 
allVehicles{3}.reach_accuracy = 'medium';
allVehicles{4}.reach_accuracy = 'medium';

% ---------------------------------------------------------------------------
% Initialize the game

% Color of the plots
fig_color = ['r', 'b', 'g', 'k'];

% Assign the sub-plot axes to the vehicles
num_col = ceil(vnum/2);
for i=1:vnum
    allVehicles{i}.mast_fig = figure; % set this equal to figure to assign different figure to each vehicle
    figure(allVehicles{i}.mast_fig);
    if(~isempty(allVehicles{i}.fig_hand))
        allVehicles{i}.fig_hand = subplot(2,num_col,i);
    end
    allVehicles{i}.fig_color = fig_color(i);
end

% Load the starting positions of the vehicle
allVehicles{1}.x(:,1) = [ -0.5, 0, 0]';
allVehicles{2}.x(:,1) = [  0.5, 0, pi]';
allVehicles{3}.x(:,1) = [ -0.6, 0.6, 7*pi/4]';
allVehicles{4}.x(:,1) = [  0.6, 0.6, 5*pi/4]';

% Also initialize the nominal trajectory to the initial state
allVehicles{1}.x_nom(:,1) = allVehicles{1}.x(:,1);
allVehicles{2}.x_nom(:,1) = allVehicles{2}.x(:,1);
allVehicles{3}.x_nom(:,1) = allVehicles{3}.x(:,1);
allVehicles{4}.x_nom(:,1) = allVehicles{4}.x(:,1);

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

% Target sets of vehicles
% Target position and target radius matrix should be provided
target_pos = zeros(vnum,2); % Number of vehicles x Number of states (to be reached)
target_pos(1,:) = [0.7,0.2];
target_pos(2,:) = [-0.7,0.2];
target_pos(3,:) = [0.7,-0.7];
target_pos(4,:) = [-0.7,-0.7];

target_radius = zeros(vnum,1);
target_radius(1,1) = 0.025;
target_radius(2,1) = 0.025;
target_radius(3,1) = 0.025;
target_radius(4,1) = 0.025;

for i=1:vnum
    allVehicles{i}.reach(:,:,:,steps+1) = sqrt((g.xs{1} - target_pos(i,1)).^2 +...
        (g.xs{2} - target_pos(i,2)).^2) - target_radius(i,1);
end

% Original target sets of the vehicles
for i=1:vnum
    allVehicles{i}.target = sqrt((g.xs{1} - target_pos(i,1)).^2 +...
        (g.xs{2} - target_pos(i,2)).^2) - 0.1;
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
        allVehicles{i} = simulate_trajectory(g, allVehicles{i});
    end
    t_start = max(t_start, allVehicles{i}.t_start);
    save('ex3', '-v7.3');
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
   
   % Adjust the reach matrix: simply delete the extra rows
   temp2 = 1e6*ones(g.shape);
   temp2  = repmat(temp2,  [ones(1,g.dim),steps+1]);
   % Extract the relevant reach matrix
   num_steps = int64((allVehicles{i}.t_end - allVehicles{i}.t_start)/t_step +1);
   start_index = int64(allVehicles{i}.t_start/t_step);
   temp2(:,:,:,start_index+1:end) = allVehicles{i}.reach(:,:,:,end-num_steps+1:end);
   allVehicles{i}.reach = temp2;
   clear('temp2');
   
   % Actual input trajectory
   allVehicles{i}.u = zeros(1,steps);

   % Applied disturbance
   allVehicles{i}.d = zeros(3,steps);

   % state trajectory (initialized to the initial state)
   temp_x = allVehicles{i}.x(:,1);
   allVehicles{i}.x = zeros(3,steps+1);
   allVehicles{i}.x(:,1:start_index+1) = repmat(temp_x, 1, start_index+1);
   
   % Fix the nominal trajectory as well
   allVehicles{i}.x_nom = [zeros(3,start_index), allVehicles{i}.x_nom];
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
            xref = allVehicles{i}.x_nom(:,index);
            vrange = [-allVehicles{i}.velocity  allVehicles{i}.velocity] + allVehicles{i}.v_nom; 
            wrange = [-allVehicles{i}.turnRate  allVehicles{i}.turnRate] + allVehicles{i}.turnRate_nom; 
            ctrl = RBControl(x_current, xref, vrange, wRange, RB);
            allVehicles{i}.u(:,index) = ctrl(2);
            allVehicles{i}.v(:,index) = ctrl(1);
            
            % Applied disturbance
            temp = [];
            allVehicles{i}.d(:,index) = applyDisturbance(allVehicles{i}, temp, 'random');
            d1 = allVehicles{i}.d(1,index);
            d2 = allVehicles{i}.d(2,index);
            d3 = allVehicles{i}.d(3,index);
            
            % Let's see if the current state is in the target set already
            location{1} = x_current(1);
            location{2} = x_current(2);
            location{3} = x_current(3);
            [location_index, ~] = getCellIndexes(g, location);
            [index1, index2, index3] = ind2sub(g.shape, location_index);
            if (allVehicles{i}.target(index1, index2, index3, end) <= 0)
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
        
%         % Process the trajectory
%         figure(allVehicles{i}.mast_fig);
%         if(~isempty(allVehicles{i}.fig_hand))
%             subplot(allVehicles{i}.fig_hand);
%         end
%         hold on,
%         plot(x_next(1), x_next(2), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
%         delete(allVehicles{i}.quiver_hand);
%         dirn = 0.2*[cos(x_next(3)) sin(x_next(3))];
%         allVehicles{i}.quiver_hand = quiver(x_next(1),x_next(2), dirn(1), dirn(2), ...
%             'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
%         drawnow;
%         axis equal;
%         hold off,
    end
    
    current_time = current_time + t_step;
    end_time = min(t_end, current_time + t_step);
    index = index+1;
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
 save('ex3', '-v7.3');
