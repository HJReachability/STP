function solveDstb(obj)
% computeAll(obj)
%     Performs all SPP with disturbance calculations using defaults in all
%     computations

obj.computeRTTRS;
obj.computeNIRS;
obj.simulateNI;
end
