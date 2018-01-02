function [initStates, tarCenters, tarCount, targetCentersSet] = ...
  gen_targets_initStates_FaSTrack(setup_name, numVeh)

switch setup_name
  case 'room'
    targetCentersSet = { ...
      [30; 40]; ...
      [8; 18]; ...
      [11; 10]; ...
      [45; 25] ...
      };
    
    initPos = [47; 20];
    initStates = cell(numVeh, 1);
    tarCenters = cell(numVeh,1);
    tarCount = zeros(length(targetCentersSet), 1);
    
    for i = 1:numVeh
      target_ind = randi(length(targetCentersSet));
      tarCount(target_ind) = tarCount(target_ind) + 1;
      
      tarCenters{i} = targetCentersSet{target_ind};
      
      targetDirection = tarCenters{i} - initPos;
      %initHeading = atan2(targetDirection(2),targetDirection(1));
      initHeading = wrapTo2Pi(atan2(targetDirection(2), targetDirection(1)));
      initVelocity = rand * 5;
      initStates{i} = [initPos; initHeading; initVelocity];
    end
    
  case 'SF'
    targetCentersSet = { ...
      [300; 400]; ...
      [50; 175]; ...
      [75; 25]; ...
      [450; 25] ...
      };
    
    initPos = [475; 200];
    initStates = cell(numVeh, 1);
    tarCenters = cell(numVeh,1);
    tarCount = zeros(length(targetCentersSet), 1);
    
    for i = 1:numVeh
      target_ind = randi(length(targetCentersSet));
      tarCount(target_ind) = tarCount(target_ind) + 1;
      
      tarCenters{i} = targetCentersSet{target_ind};
      
      targetDirection = tarCenters{i} - initPos;
      %initHeading = atan2(targetDirection(2),targetDirection(1));
      initHeading = wrapTo2Pi(atan2(targetDirection(2), targetDirection(1)));
      initVelocity = rand * 5;
      initStates{i} = [initPos; initHeading; initVelocity];
    end
    
  case 'Bay_Area'
end

end