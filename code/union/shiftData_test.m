function shiftData_test()

addpath('..')

[g, data] = computeBaseBRS;

N = 9;
spC = ceil(sqrt(N));
spR = ceil(N/spC);
shifts = -2 + 4*rand(N, 2);

figure
for i = 1:N
  dataRot = shiftData(g, data(:,:,:,end), shifts(i,:));
  
  subplot(spR, spC, i)
  visSetIm(g, dataRot);
  title(['shift = [' num2str(shifts(i, 1)) ' ' num2str(shifts(i, 2)) ']'])
  axis square
  view(2)
end
end