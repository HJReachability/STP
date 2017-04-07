function p = loadSPPProblem(problem_name, extraArgs)
      if nargin < 2
        extraArgs = [];
      end
      if isfield(extraArgs, 'SPPP_filename')
           p = SPPProblem(problem_name, extraArgs);
           filename = extraArgs.SPPP_filename;
           load(filename, 'SPPP');
           p.Rc = getVarFromStruct(SPPP, 'Rc');
           p.buffer_duration = getVarFromStruct(SPPP, 'buffer_duration');
           p.buffer_duration_ind = getVarFromStruct(SPPP, 'buffer_duration_ind');
           p.dMaxA = getVarFromStruct(SPPP, 'dMaxA');
           p.dt = getVarFromStruct(SPPP, 'dt');
           p.g = cpp2matG(SPPP.g);
           p.g2D = cpp2matG(SPPP.g2D);
           p.gMin = getVarFromStruct(SPPP, 'gMin');
           p.gMax = getVarFromStruct(SPPP, 'gMax');
           p.gN = getVarFromStruct(SPPP, 'gN');
            p.gN = p.gN';
           p.initStates = getVarFromStruct(SPPP, 'initStates');
           p.max_num_affected_vehicles = getVarFromStruct(SPPP, 'max_num_affected_vehicles');
           p.remaining_duration_ind = getVarFromStruct(SPPP, 'remaining_duration_ind');
           p.staticObs = getVarFromStruct(SPPP, 'staticObs');
           p.augStaticObs = getVarFromStruct(SPPP, 'augStaticObs');
           p.tIAT = getVarFromStruct(SPPP, 'tIAT');
            p.tIntr = getVarFromStruct(SPPP, 'tIntr');
            p.tMin = getVarFromStruct(SPPP, 'tMin');
            p.tTarget = getVarFromStruct(SPPP, 'tTarget');
                p.tReplan = getVarFromStruct(SPPP, 'tReplan');
            p.targetCenters = getVarFromStruct(SPPP, 'targetCenters');
            p.targetR = getVarFromStruct(SPPP, 'targetR');
            p.targetRsmall = getVarFromStruct(SPPP, 'targetRsmall');
            p.tau = getVarFromStruct(SPPP, 'tau');
            p.tauBR = getVarFromStruct(SPPP, 'tauBR')
            p.tauAR = getVarFromStruct(SPPP, 'tauAR');
            p.tauSim = getVarFromStruct(SPPP, 'tauSim');
            p.vRangeA = getVarFromStruct(SPPP, 'vRangeA');
            p.vRangeA = p.vRangeA';
            p.wMaxA = getVarFromStruct(SPPP, 'wMaxA');
      end
    
      SPPP = p;
      save(sprintf('%s/SPPP.mat', p.folder), 'SPPP', '-v7.3');
      
%      p.plotSetup();
end
