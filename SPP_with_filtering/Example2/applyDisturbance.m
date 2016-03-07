function disturbance = applyDisturbance(vehicle, p, type)

if(strcmp(type,'random'))
   disturbance = (vehicle.disturbance_mag).*(-1 + 2*rand(3,1));
elseif(strcmp(type,'worst'))
    disturbance = [(vehicle.disturbance_mag).*sign(p')];
end