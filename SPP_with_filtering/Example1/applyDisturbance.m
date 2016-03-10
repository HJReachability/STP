function disturbance = applyDisturbance(vehicle, p, type)

% if(strcmp(type,'random'))
%    disturbance = (vehicle.disturbance_mag).*(-1 + 2*rand(3,1));
% elseif(strcmp(type,'worst'))
%     disturbance = [(vehicle.disturbance_mag).*sign(p')];
% end

if(strcmp(type,'random'))
    % Now create the set of points.
    t = 2*pi*rand(1);
    r = vehicle.disturbance_mag(1)*sqrt(rand(1));
    d1 = r*cos(t);
    d2 = r*sin(t);
    d3 = vehicle.disturbance_mag(3)*(-1 + 2*rand(1));
    disturbance = [d1; d2; d3];
elseif(strcmp(type,'worst'))
    d1 = vehicle.disturbance_mag(1) * (p(1)/sqrt(p(1)^2 + p(2)^2));
    d2 = vehicle.disturbance_mag(1) * (p(2)/sqrt(p(1)^2 + p(2)^2));
    d3 = vehicle.disturbance_mag(3) * sign(p(3));
    disturbance = [d1; d2; d3];
end