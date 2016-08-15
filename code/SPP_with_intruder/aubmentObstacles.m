function vehicle = aubmentObstacles(vehicle, schemeData, rawObs)
% vehicle = computeBaseObs(vehicle, schemeData, resetR)
%     Computes the induced base obstacles by vehicle according to BRS1 and the
%     centralized controller scheme; populates the .baseObs and .baseObs_tau
%     fields of vehicle.data
%
% Inputs
%     vehicle:
%         vehicle object; should have the .data field populated with .BRS1 and
%         .BRS1_tau
%     schemeData:
%         parameters for the HJI PDE solver; should contain the fields .grid and
%         .dynSys
%     resetR:
%         minimum size for the reachable set during evolution; if the reachable
%         set goes below this size, the reachable set will be propagated
%         "maximally" until the size becomes large enough
%
% Output
%     vehicle:
%         updated vehicle object with .baseObs and .baseObs_tau populated in the
%         vehicle.data field

for i = 1:length(nomTraj_tau)
    % Rotate and shift the robust trajectory tracking reachable set to the vehicle
    % state
    RTTRS = rotateData(schemeData.grid, params.RTTRS, vehicle.x(3), [1 2], 3);
    RTTRS = shiftData(schemeData.grid, RTTRS, vehicle.x([1 2]), [1 2]);
    vehicle.data.baseObs(:,:,:,obsInd) = RTTRS;
    
    % Update nominal trajectory
    vehicle.data.nomTraj(:,obsInd) = vehicle.x;
  end
end


end