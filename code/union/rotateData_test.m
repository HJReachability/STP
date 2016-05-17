function rotateData_test()

addpath('..')

[g, data] = computeBaseBRS;

N = 9;
spC = ceil(sqrt(N));
spR = ceil(N/spC);
thetas = linspace(0, 2*pi, N);

figure
for i = 1:N
  dataRot = rotateData(g, data(:,:,:,end), thetas(i));
  
  subplot(spR, spC, i)
  visSetIm(g, dataRot);
  title(['\theta = ' num2str(thetas(i))])
  axis square
  view(2)
end


end