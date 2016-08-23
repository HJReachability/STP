function computeAll(obj)
% computeAll(obj)
%     Performs all SPP with intruder calculations using defaults in all
%     computations

obj.computeRTTRS;
obj.computeCARS;
obj.computeRawAugObs;
obj.computeBRRS;
obj.simulateBR;
obj.computeARRS;
obj.simulateAR;
obj.simulateFull;
end
