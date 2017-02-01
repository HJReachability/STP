function simulate_bufferRegion_properties(obj)

%% Load files
if exist(obj.CARS_filename, 'file')
  fprintf('Loading CARS...\n')
  load(obj.CARS_filename)
else
  error('CARS file not found!')
end

if exist(obj.minMinBRS_filename, 'file')
  fprintf('Loading minMinBRS...\n')
  load(obj.minMinBRS_filename)
else
  error('minMinBRS file not found!')
end

avoid_wMax = CARS.dynsys.wMaxA;
avoid_vRange = CARS.dynsys.vRangeA;

intr_wMax = CARS.dynsys.wMaxB;
intr_vRange = CARS.dynsys.vRangeB;

if avoid_wMax ~= minMinBRS.dynsys.wMaxA;
  error('CARS and minMinBRS wMaxA do not agree!')
end

if any(avoid_vRange ~= minMinBRS.dynsys.vRangeA);
  error('CARS and minMinBRS vRangeA do not agree!')
end

if intr_wMax ~= minMinBRS.dynsys.wMaxB;
  error('CARS and minMinBRS wMaxB do not agree!')
end

if any(intr_vRange ~= minMinBRS.dynsys.vRangeB);
  error('CARS and minMinBRS vRangeB do not agree!')
end

x01 = [0; 0; 0];
pl1 = Plane(x01, avoid_wMax, avoid_vRange);
% pl2 = Plane(x02, avoid_wMax, avoid_vRange);
% pl3 = Plane(x03, avoid_wMax, avoid_vRange);

x0I = [10; 10; -3*pi/4];
% plI = Plane(x0I, intr_wMax, intr_vRange);

dt = 0.1;
tMax = 10;
t = 0:dt:tMax;

figure;
[g2D, data2D] = proj(CARS.g, CARS.data(:,:,:,end), [0 0 1], x0I(3));
visSetIm(g2D, data2D, 'r');

end