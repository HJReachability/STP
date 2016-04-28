for i=205:330
   radius(i) = sqrt(obstacles{i}.a ^2 + obstacles{i}.b^2);
   cenx(i) = obstacles{i}.cenx;
   ceny(i) = obstacles{i}.ceny;
   dirn(i) = obstacles{i}.dirn;
end

figure,
plot(radius, 'color', 'r');
hold on;
plot(dirn, 'color', 'b');

figure,
plot(cenx, ceny, 'color', 'b');
   