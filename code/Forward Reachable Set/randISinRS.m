function IS = randISinRS(BRS)

x = BRS.g.min(1) + (BRS.g.max(1)-BRS.g.min(1))*rand;
y = BRS.g.min(2) + (BRS.g.max(2)-BRS.g.min(2))*rand;
theta = 2*pi*rand;
ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);

while ival > 0
  disp(['Random value: ' num2str(ival) '; retrying...'])
  x = -10 + 20*rand;
  y = -10 + 20*rand;
  theta = 2*pi*rand;
  ival = eval_u(BRS.g, BRS.data(:,:,:,end), [x; y; theta]);
end
disp(['Initial value: ' num2str(ival)])

IS = [x; y; theta];
end