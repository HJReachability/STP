function computeNomTraj(obj, SPPP)
% Computes nominal trajectory of to be robustly tracked

% Variables needed from SPPP class
g = SPPP.g;
dt = SPPP.dt;

subSamples = 4;
dtSmall = dt/subSamples;

% Save state and control histories
x = obj.x;
xhist = obj.xhist;
u = obj.u;
uhist = obj.uhist;

% Modify control bounds
vrange = obj.vrange;
wMax = obj.wMax;
obj.vrange = vrange + obj.vReserved;
obj.wMax = wMax + obj.wReserved;

% No disturbance when computing trajectory
d = [0; 0; 0];

% Save nominal trajectory if it already exists
oldNomTraj = obj.nomTraj;
oldNomTraj_tau = obj.nomTraj_tau;

% Initialize nominal trajectory
obj.nomTraj_tau = obj.BRS1_tau;
obj.nomTraj = nan(3, length(obj.BRS1_tau));
obj.nomTraj(:,1) = obj.x;

% Compute trajectory
small = 1e-4;

t = 1;
nomTraj_t = 2;

folder = sprintf('nomTraj_%f', now);
system(sprintf('mkdir %s', folder));

figure
while t < length(obj.BRS1_tau)
  % Determine the earliest time that the current state is in the reachable set
  for tEarliest = length(obj.BRS1_tau):-1:t
    valueAtX = eval_u(g, obj.BRS1(:,:,:,tEarliest), obj.x);
    if valueAtX < small
      break
    end
  end
  
  % Compute gradient and integrate trajectory
  BRS1t = obj.BRS1(:,:,:,tEarliest);
  Deriv = computeGradients(g, BRS1t);
  for j = 1:subSamples
    deriv = eval_u(g, Deriv, obj.x);
    u = obj.optCtrl([], obj.x, deriv, 'min');
    obj.updateState(u, dtSmall, obj.x, d);
  end
  
  % Record new point on nominal trajectory
  obj.nomTraj(:,nomTraj_t) = obj.x;
  
  % Plot
  plot(obj.nomTraj(1,nomTraj_t), obj.nomTraj(2,nomTraj_t), 'k.')
  hold on
  [g2D, data2D] = proj(g, BRS1t, [0 0 1], obj.nomTraj(3,nomTraj_t));
  visSetIm(g2D, data2D);
  export_fig(sprintf('%s/%d', folder, nomTraj_t), '-png')
  hold off
  
  % Update time stamps
  nomTraj_t = nomTraj_t + 1;
  t = tEarliest + 1;  
end

% Delete unused indices
obj.nomTraj_tau(nomTraj_t:end) = [];
obj.nomTraj(:,nomTraj_t:end) = [];

% Pad based on FRS1_tau, if available
if ~isempty(obj.FRS1)
  indsBeforeBRS1 = find(obj.FRS1_tau < min(obj.BRS1_tau)-small);
  pad_tau = obj.FRS1_tau(indsBeforeBRS1);
  pad_nomTraj = repmat(obj.nomTraj(:,1), 1, length(indsBeforeBRS1));
  
  obj.nomTraj_tau = [pad_tau obj.nomTraj_tau];
  obj.nomTraj = [pad_nomTraj obj.nomTraj];
end

% Attach old reachable set if needed
keepInds = find(oldNomTraj_tau < min(obj.nomTraj_tau) - small);
obj.nomTraj_tau = [oldNomTraj_tau(keepInds) obj.nomTraj_tau];
obj.nomTraj = [oldNomTraj(:,keepInds) obj.nomTraj];

% Undo control bounds modification
obj.vrange = vrange;
obj.wMax = wMax;

% Undo state and control information
obj.x = x;
obj.xhist = xhist;
obj.u = u;
obj.uhist = uhist;
end