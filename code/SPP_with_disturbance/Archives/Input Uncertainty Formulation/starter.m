%% To dos
%  - Complete the load data section appropriately
%  - Implement the new collision calculation function:
%     - Check the structure of collision matrix again
%     - See what to do with the nominal trajectory variable u_vom
%     - Take the 2-D projection of a 3-D obstacle and then extend it in the
%       third dimension
%       - Ask Mo if proj2D does that. If so, understand how to use it.
%  - Implement the figure functions

%% Function starts here

clear all;
close all;
f1 = figure;

%---------------------------------------------------------------------------
% Grid of joint space
Nx = 51;

% Create the computation grid.
g.dim = 3;
g.min = [  -1; -1; 0];
g.max = [ +1; +1; 2*pi];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx];

g = processGrid(g);

%---------------------------------------------------------------------------
% Time parameters
t_start = 0;
t_end = 0.1;
t_step = 0.01;
steps = ceil((t_end - t_start)/t_step);

%---------------------------------------------------------------------------
% Create default vehicle object
% There should be one vehicle object for each vehicle

% Integration parameters
vehicle.accuracy = 'medium';
vehicle.t_step = t_step; % can also be different for obstacle simulation
vehicle.capture_radius = 0.1;

% Other vehicle parameters
vehicle.velocity = 1;
vehicle.turnRate = 1;

% State uncertainty models: circular, ellipsoidal
% vehicle.state_uncertainty = 'circular';
% vehicle.state_uncertainty_axis = 0.4;

vehicle.state_uncertainty = 'ellipsoid';
vehicle.state_uncertainty_axis = [0.1, 0.1, 0.1];

% Input uncertainty models: box
vehicle.input_uncertainty = 'box';
vehicle.input_uncertainty_axis = 0.8; %assuming symmetric lower and upper bounds

% Define and initialize collisionmat matrix
% This matrix contains the most conservative estimate of the vehicle
% positions
vehicle.collisionmat = -1e6*ones(g.shape);
vehicle.collisionmat  = repmat(vehicle.collisionmat,  [ones(1,g.dim),steps]);

% Define and initialize reachable set matrix
vehicle.reach = 1e6*ones(g.shape);
vehicle.reach  = repmat(vehicle.reach,  [ones(1,g.dim),steps+1]);

% Nominal input trajectory
vehicle.u_nom = 0.5*ones(1,steps);

% Actual input trajectory
vehicle.actual_u = 0.45*ones(1,steps);

% Current state (initialized to the initial state)
vehicle.current_x = zeros(1,3);

% Current state measurement
vehicle.mment = zeros(1,3);

%---------------------------------------------------------------------------
% Create an object for each vehicle
vnum = 2; % total number of vehicles
for i=1:vnum
    allVehicles{i} = vehicle;
end

% ---------------------------------------------------------------------------
%% Load the game data

% Load the nominal input of all vehicles

% Assign the sub-plot axes to the vehicles
num_col = ceil(vnum/2);
fig_color = ['r', 'b', 'g', 'k'];
for i=1:vnum
    allVehicles{i}.mast_fig = f1;
    allVehicles{i}.fig_hand = subplot(2,num_col,i);
    allVehicles{i}.fig_color = fig_color(i);
end

% Load the starting positions of the vehicle
allVehicles{1}.current_x = [0, 0, pi];
allVehicles{2}.current_x = [0, 0, pi];

% Load the measurements for the first vehicle (in a temp matrix, mment_temp,  maybe)
mment_temp = repmat([0, 0, pi],steps+1,1);

% Create the first measurement for all other vehicles
factor = 0.7;
for i=2:vnum
    allVehicles{i}.mment = allVehicles{i}.current_x + factor*rand(1,3).*allVehicles{i}.state_uncertainty_axis;
end
allVehicles{1}.mment = mment_temp(1,:);

% Plot the initial positions of the vehicles
figure(f1);
for i=1:vnum
    subplot(allVehicles{i}.fig_hand);
    axis equal;
    plot(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), 'Linestyle', '-.', 'color', allVehicles{i}.fig_color);
    hold on,
    dirn = 0.2*[cos(allVehicles{i}.current_x(3)) sin(allVehicles{i}.current_x(3))];
    allVehicles{i}.quiver_hand = quiver(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), dirn(1), dirn(2), ...
        'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
    drawnow;
    hold off,
end

% Update the target set for each vehicle
% Target position and target radius matrix should be provided
target_pos = zeros(vnum,2); % Number of vehicles x Number of states (to be reached)
target_pos(1,:) = [0.9,0.9];
target_pos(2,:) = [0.5,0.5];

target_radius = zeros(vnum,1);
target_radius(1,1) = 0.05;
target_radius(2,1) = 0.05;

for i=1:vnum
    allVehicles{i}.reach(:,:,:,steps+1) = sqrt((g.xs{1} - target_pos(i,1)).^2 +...
        (g.xs{2} - target_pos(i,2)).^2) - target_radius(i,1);
end

% Plot the target sets of the vehicles
figure(f1);
for i=1:vnum
    subplot(allVehicles{i}.fig_hand);
    hold on,
    [g2D, data2D] = proj2D(g, allVehicles{i}.reach(:,:,:,end), [0 0 1], 0);
    contour(g2D.xs{1},g2D.xs{2}, data2D, [0 0], 'color', allVehicles{i}.fig_color,'linewidth',2);
    drawnow;
    hold off,
end

% Number of obstacle update steps
OU = 4;


% % Load the measurement matrix: it should be a steps*num_states*vnum size
% % matrix, where:
% % vnum: number of total vehicles
% % num_states: number of states of each vehicle
% % steps: number of time steps
% % The 1st entry in the measurement matrix for each vehicle represents the
% % initial state of the vehicle
% temp = linspace(0,0.1,steps) ;
% mment = [temp', -temp', pi+zeros(steps,1)];
% mment  = repmat(mment,  [1,1,vnum]);



% ---------------------------------------------------------------------------
%% Initialize the obstacle shapes-- most conservative estimates of obstacles
index = 1;
% Last vehicle won't be an obstacle for any vehicle so no need to find
% obstacles corresponding to that vehicle
for i=1:vnum-1
    allVehicles{i} = updateCollisionObs(g, t_start, t_end, index, allVehicles{i});
end
allVehicles{vnum}.collisionmat = [];

% Now remember that since the nominal input of higher priority vehicles is
% being communicated to the other vehicles, they can only deviate around
% the nominal input, whereas the lowest priority vehicle is free to choose
% from the entire range of inputs and adjust for the uncertainty. Also note
% that this whole problem starts either by uncertainty in the position of
% the first vehicle and/or some vehicle deviating from its promised nominal
% input, although still respecting the uncertainty bounds. Therefore,

allVehicles{vnum}.input_uncertainty_axis = ...
    allVehicles{vnum}.input_uncertainty_axis + allVehicles{vnum}.turnRate;
allVehicles{vnum}.u_nom = zeros(size(allVehicles{vnum}.u_nom));

% Initialize the reachable set-- based on the most conservative obstacle
% estimates
% No reason to re-compute the reachable set for vehicle 1
for i=2:vnum
    allVehicles{i} = updateReachSet(g, t_start, t_end, index, allVehicles{i}, allVehicles(1:i-1));
    
    % Process the input
    x_current = allVehicles{i}.current_x;
    P = extractCostates(g,allVehicles{i}.reach(:,:,:,index));
    p = calculateCostate(g,P,x_current);
    allVehicles{i}.actual_u(index) = allVehicles{i}.u_nom(index) - allVehicles{i}.input_uncertainty_axis*sign(p(3));
    
    % Actual next state
    x_next(1) = x_current(1) + allVehicles{i}.velocity*cos(x_current(3))*t_step;
    x_next(2) = x_current(2) + allVehicles{i}.velocity*sin(x_current(3))*t_step;
    x_next(3) = x_current(3) + allVehicles{i}.actual_u(i)*t_step;
    allVehicles{i}.current_x = x_next;
    
    % Change the line style of the most conservative reachable set
    set(allVehicles{i}.last_hand, 'Linestyle', ':');
    
    % Process the trajectory
    subplot(allVehicles{i}.fig_hand);
    hold on,
    plot(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
    delete(allVehicles{i}.quiver_hand);
    dirn = 0.2*[cos(allVehicles{i}.current_x(3)) sin(allVehicles{i}.current_x(3))];
    allVehicles{i}.quiver_hand = quiver(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), dirn(1), dirn(2), ...
        'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
    drawnow;
    hold off,
    
    % Generate the measurement
    allVehicles{i}.mment = allVehicles{i}.current_x + factor*rand(1,3).*allVehicles{i}.state_uncertainty_axis;
end


%---------------------------------------------------------------------------
%% Now update the obstacle shapes and compute the reachable sets on the fly
current_time = t_start + t_step;
end_time = min(t_end, current_time + OU*t_step);
index = index+1;
allVehicles{1}.mment = mment_temp(index,:);

while(current_time < t_end)
    % Update collision set
    for i=1:vnum-1
        allVehicles{i} = updateCollisionObs(g, current_time, end_time, index, allVehicles{i});
    end
    
    % Update reacahble set
    for i=2:vnum
        if(index ~= 2)
            delete(allVehicles{i}.last_hand);
        end
        allVehicles{i} = updateReachSet(g, current_time, end_time, index, allVehicles{i}, allVehicles(1:i-1));
        
        % Process the input
        x_current = allVehicles{i}.current_x;
        P = extractCostates(g,allVehicles{i}.reach(:,:,:,index));
        p = calculateCostate(g,P,x_current);
        allVehicles{i}.actual_u(index) = allVehicles{i}.u_nom(index) - allVehicles{i}.input_uncertainty_axis*sign(p(3));
        
        % Actual next state
        x_next(1) = x_current(1) + allVehicles{i}.velocity*cos(x_current(3))*t_step;
        x_next(2) = x_current(2) + allVehicles{i}.velocity*sin(x_current(3))*t_step;
        x_next(3) = x_current(3) + allVehicles{i}.actual_u(i)*t_step;
        allVehicles{i}.current_x = x_next;
        
        % Process the trajectory
        subplot(allVehicles{i}.fig_hand);
        hold on,
        plot(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), 'marker', 'o', 'color', allVehicles{i}.fig_color,'markersize',5);
        delete(allVehicles{i}.quiver_hand);
        dirn = 0.2*[cos(allVehicles{i}.current_x(3)) sin(allVehicles{i}.current_x(3))];
        allVehicles{i}.quiver_hand = quiver(allVehicles{i}.current_x(1), allVehicles{i}.current_x(2), dirn(1), dirn(2), ...
            'maxheadsize', 50, 'marker', '*', 'color', allVehicles{i}.fig_color);
        hold off,
        
        % Generate the measurement
        allVehicles{i}.mment = allVehicles{i}.current_x + factor*rand(1,3).*allVehicles{i}.state_uncertainty_axis;
    end
    
    current_time = current_time + t_step;
    end_time = min(t_end, current_time + OU*t_step);
    index = index+1;
    allVehicles{1}.mment = mment_temp(index,:);
end

