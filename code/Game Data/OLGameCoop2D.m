%---------------------------------------------------------------------------
% DYNAMICS
captureRadius = 0.1;
velocity1 = 1;
velocity2 = 1;

dims_1 = [1 1 0 0];
dims_2 = [0 0 1 1];

% 2D version of grid
N2D = 200;
g2D = proj2D(g, dims_1, N2D);

%---------------------------------------------------------------------------
% DOMAIN
% [dom_bdry,dom_map] = std_domain(g2D);
[dom_bdry,dom_map] = OLGameModified_domain(g2D);

%---------------------------------------------------------------------------
% TARGET SET
target1.xmin = 0.6;
target1.xmax = 0.8;
target1.ymin = 0.1;
target1.ymax = 0.3;
target1.type = 'rect';

target14D = shapeRectangleByCorners(g,[target1.xmin target1.ymin -inf -inf], ...
    [target1.xmax target1.ymax inf inf]);
target12D = createShape2D(g2D,target1);

target2.xmin = -0.8;
target2.xmax = -0.6;
target2.ymin = 0.1;
target2.ymax = 0.3;
target2.type = 'rect';

target24D = shapeRectangleByCorners(g,[-inf -inf target2.xmin target2.ymin], ...
    [inf inf target2.xmax target2.ymax]);
target22D = createShape2D(g2D,target2);

target4D = shapeIntersection(target14D, target24D);
target2D = shapeUnion(target12D, target22D);
%---------------------------------------------------------------------------
% OBSTACLES
% Parameters
obs1.xmin = -0.1;
obs1.xmax = 0.1;
obs1.ymin = -inf;
obs1.ymax = -0.3;
obs1.type = 'rect';

obs2.xmin = -0.1;
obs2.xmax = 0.1;
obs2.ymin = 0.3;
obs2.ymax = 0.6;
obs2.type = 'rect';

% obs.xmin = -0.1;
% obs.xmax = 0.1;
% obs.ymin = 0.3;
% obs.ymax = 0.6;
% obs.type = 'rect';

% 4D Obstacles
[obs1_1, obs1_2] = createObstacle4D(g, obs1);
[obs2_1, obs2_2] = createObstacle4D(g, obs2);
obs_1 = shapeUnion(obs1_1, obs2_1);
obs_2 = shapeUnion(obs1_2, obs2_2);
obs4D = shapeUnion(obs_1, obs_2);
% [obs_a, obs_d] = createObstacle4D(g, obs);

% 2D Obstacles
% clrc = 0.05;
% obs1a = createShape2D(g2D,obs1,clrc);
% obs2a = createShape2D(g2D,obs2,clrc);
% obs_clrc = shapeUnion(obs1a, obs2a);
% 
obs1 = createShape2D(g2D, obs1);
obs2 = createShape2D(g2D, obs2);
obs2D = shapeUnion(obs1, obs2);
% obs2D = createShape2D(g2D,obs);
% clrc = 0.05;
% obs_clrc = createShape2D(g2D, obs2D, clrc);


%---------------------------------------------------------------------------
% PLAYER PARAMETERS
x1_init = cell(1,1);
x1_init{1} = [-0.5 0];
% x1_init{1} = [0.0206    0.1398];
% x1_init{1} = [0.2529    0.2948];

N1 = length(x1_init);

x2_init = cell(1,1);
x2_init{1} = [0.5 0];
% x2_init{1} = [ -0.0114   -0.1274];
% x2_init{1} = [-0.3337    0.0074];
N2 = length(x2_init);

% ----------------------------------------------------------------------------
targetLoc = target4D;

% - Defender has not captured the attacker
% Create capture condition, this is also part of the avoid set
collision = (g.xs{1} - g.xs{3}) .^2 + (g.xs{2} - g.xs{4}) .^2;
collision = sqrt(collision) - captureRadius;

targetLoc = shapeDifference(targetLoc, collision);

