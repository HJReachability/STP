clearvars -except vnum
% close all

%---------------------------------------------------------------------------
% Grid of joint space
Nx = 71;

% Create the computation grid.
g.dim = 3;
g.min = [  -1; -1; 0];
g.max = [ +1; +1; 2*pi];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate; @addGhostPeriodic};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx; Nx];

g = processGrid(g);

vnum = 4;
run('Game Data\OLGameCoop3D')
data_dir = 'F:\Research\ECC 2015 data';
load([data_dir '/coop3D_' num2str(vnum)])
%---------------------------------------------------------------------------
% Integrate trajectory
% dt = 0.01;
% Tf = 2.5;

x = x_init{vnum};
traj = x;
u_seq = [];

f1 = figure;
contour(g2D.xs{1}, g2D.xs{2}, target_2D{vnum}, [0 0], 'r'); hold on
contour(g2D.xs{1}, g2D.xs{2}, obs_2D, [0 0], 'k');
hx = plot(x(1),x(2),'r.','markersize',10);
hxt = plot(traj(:,1),traj(:,2),'r:','markersize',10);

[~, obstacle_2D] = proj2D(g, [0 0 1], N2D, obstacle(:,:,:,1), 0);
[~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obstacle_2D, [0 0], 'k');

title(['t=' num2str(tau(1))])
axis square
drawnow;

for i = 1:length(tau)-1
    % Optimal control
    P = extractCostates(g,reach(:,:,:,i));
    p = calculateCostate(g,P,x);
    w = -sign(p(3));
    
    % Update state
    dt = tau(i+1) - tau(i);
    x(1) = x(1) + velocity*cos(x(3))*dt;
    x(2) = x(2) + velocity*sin(x(3))*dt;
    x(3) = x(3) + turnRate*w*dt;

    if x(3) < 0, x(3) = x(3) + 2*pi; end
    if x(3) > 2*pi, x(3) = x(3) - 2*pi; end

    u_seq = cat(1, u_seq, w);
    traj = cat(1, traj, x); % Append to trajectory matrix
    
    % Plotting
    delete(hx); delete(hxt); delete(ho);
    hx = plot(x(1),x(2),'r.','markersize',10);
    hxt = plot(traj(:,1), traj(:,2), 'r:');
    
    [~, obstacle_2D] = proj2D(g, [0 0 1], N2D, obstacle(:,:,:,i), 0);
    [~, ho] = contour(g2D.xs{1}, g2D.xs{2}, obstacle_2D, [0 0], 'k');
    
    title(['t=' num2str(tau(i+1))])
    drawnow;
    
    % Has target been reached?
    if eval_u(g, target_3D{vnum}, x) < 0
        disp('Destinations reached!')
        break;
    end
end

tau(i+2:end) = [];
% keyboard
save([data_dir '/coop3D_' num2str(vnum)], 'u_seq','-append')