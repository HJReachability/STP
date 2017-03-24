function Q = loadNIRS(filename, gN, clearLargeData)
if nargin < 3
  clearLargeData = true;
end
  load(filename, 'Q');
  for  i=1:length(Q)
    fprintf('Loading %dth vehicle...\n', i)
    Q{i} = cpp2matSPPPlane(Q{i}, gN);
    % remove large unused data.
    if clearLargeData
        Q{i} .obs2D = [];
        Q{i} .BRS1 = [];
    end
  end
end
