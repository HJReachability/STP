function IntruderRTT_large_test(SPPP)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 1
  theta = 5*pi/4;
  ringRadius = 4.5;
  initStates = {[ringRadius*cos(theta); ringRadius*sin(theta); theta + pi] };
  
  targetCenters = {[ringRadius*cos(theta+pi); ringRadius*sin(theta+pi); 0]};
  
  targetR = 0.15;
  
  % Vehicle parameters
  vehParams.vRangeA = [0.1 1];
  vehParams.wMaxA = 1;
  vehParams.dMaxA = [0.1 0.2];
  
  % Grid parameters
  grid_params.min = [-5; -5; 0];
  grid_params.max = [5; 5; 2*pi];
  grid_params.N = [301; 301; 95];
  
  SPPP = SPPProblem(initStates, targetCenters, targetR, vehParams, grid_params);
  fprintf('Enter any modifications to the SPPProblem...\n')
  keyboard
end

% RTT parameters
vReserved = [0.4 -0.2];
wReserved = -0.4;
trackingRadius = 0.075;

SPPP.computeRTTRS(vReserved, wReserved, trackingRadius);
SPPP.computeCARS;
SPPP.computeRawAugObs;
SPPP.computeBRRS;
SPPP.simulateBR([4.5; -4.5; 0], [0; 0]);

end