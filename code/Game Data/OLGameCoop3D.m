%---------------------------------------------------------------------------
% DYNAMICS
captureRadius = 0.1;
velocity = 1;
turnRate = 1;

dims_p = [0 0 1];


% 2D version of grid
N2D = 200;
g2D = proj2D(g, dims_p, N2D);

%---------------------------------------------------------------------------
% DOMAIN
% [dom_bdry,dom_map] = std_domain(g2D);
[dom_bdry,dom_map] = OLGameModified_domain(g2D);

%---------------------------------------------------------------------------
% TARGET SET
target1.center = [0.7 0.2];
target1.radius = 0.1;
target1.type = 'circ';

target_3D{1} = shapeCylinder(g, 3, target1.center, target1.radius);
target_2D{1} = createShape2D(g2D, target1);

target2.center = [-0.7 0.2];
target2.radius = 0.1;
target2.type = 'circ';

target_3D{2} = shapeCylinder(g, 3, target2.center, target2.radius);
target_2D{2} = createShape2D(g2D, target2);

target3.center = [0.7 -0.7];
target3.radius = 0.1;
target3.type = 'circ';

target_3D{3} = shapeCylinder(g, 3, target3.center, target3.radius);
target_2D{3} = createShape2D(g2D, target3);

target4.center = [-0.7 -0.7];
target4.radius = 0.1;
target4.type = 'circ';

target_3D{4} = shapeCylinder(g, 3, target4.center, target4.radius);
target_2D{4} = createShape2D(g2D, target4);

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

% 3D Obstacles
obs13D = shapeRectangleByCorners(g, [obs1.xmin obs1.ymin -inf], ...
    [obs1.xmax obs1.ymax inf]);

obs23D = shapeRectangleByCorners(g, [obs2.xmin obs2.ymin -inf], ...
    [obs2.xmax obs2.ymax inf]);

obs_3D = shapeUnion(obs13D, obs23D);

% 2D Obstacles
obs1 = createShape2D(g2D, obs1);
obs2 = createShape2D(g2D, obs2);
obs_2D = shapeUnion(obs1, obs2);


%---------------------------------------------------------------------------
% PLAYER PARAMETERS
x_init = cell(1,1);
x_init{1} = [-0.5 0 0];
x_init{2} = [0.5 0 pi];
x_init{3} = [-0.6 0.6 7*pi/4];
x_init{4} = [0.6 0.6 5*pi/4];

tf{1} = 0;
tf{2} = 0.2;
tf{3} = 0.4;
tf{4} = 0.6;

N = length(x_init);

