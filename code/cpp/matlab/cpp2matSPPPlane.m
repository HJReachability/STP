function p = cpp2matSPPPlane(cppObj, gN)
    x = getVarFromStruct(cppObj,'x');
    wMax = getVarFromStruct(cppObj,'wMax');
    vrange = getVarFromStruct(cppObj,'vrange');
    dMax = getVarFromStruct(cppObj,'dMax');
    p = SPPPlane(x, wMax, vrange, dMax);
    p.BRS1_tau = getVarFromStruct(cppObj,'BRS1_tau');
    BRS1 = getVarFromStruct(cppObj, 'BRS1');
    if iscell(BRS1)
        gBRS1N = [gN(1) gN(2) gN(3)];
        clns = repmat({':'}, 1, length(gN));
        if length(BRS1) >= 1
	  p.BRS1 = zeros(size(BRS1{1}), length(BRS1), 'single');
        end
        for  t=1:length(BRS1)
            if ~isempty(BRS1{t})
               tmp = BRS1{t};
               hoge = reshape(tmp(:), gBRS1N);
               p.BRS1(clns{:},t) = hoge;
               BRS1{t} = [];
            end
        end
  	clearvars BRS1;
    end
    p.BRS1_tau = p.BRS1_tau';
    p.FRS1_g = getVarFromStruct(cppObj, 'FRS1_g');
    hdim = getVarFromStruct(cppObj, 'hdim');
    p.hdim = hdim+1;
    p.hpv = getVarFromStruct(cppObj, 'hpv');
    p.hpvhist = getVarFromStruct(cppObj, 'hpvhist');
    p.nd = getVarFromStruct(cppObj, 'nd');
    p.nu = getVarFromStruct(cppObj, 'nu');
    p.nx = getVarFromStruct(cppObj, 'nx');
    nomTraj = getVarFromStruct(cppObj, 'nomTraj');
    for t=1:length(nomTraj)
        p.nomTraj(:,t) = nomTraj{t};
    end
    clearvars nomTraj;
    nomTraj_tau = getVarFromStruct(cppObj, 'nomTraj_tau');
    p.nomTraj_tau = nomTraj_tau';
    obs2D_tau = getVarFromStruct(cppObj, 'obs2D_tau');
    p.obs2D_tau = obs2D_tau';
    obs2D = getVarFromStruct(cppObj, 'obs2D');
%    gN2D = [gN(1) gN(2) length(obs2D_tau)];
    gN2D = [gN(1) gN(2)];
    if length(obs2D) >= 1
        p.obs2D = zeros(gN(1), gN(2), length(obs2D), 'single');
    end
    clns2D = repmat({':'}, 1, length(gN2D));
    for t=1:length(obs2D)
        obs = obs2D{t};
        obs2D{t} = [];
        obs = single(obs);
        obs = reshape(obs(:), gN2D);
        p.obs2D(clns2D{:},t) = obs;
%        for i=1:gN(2)
%            p.obs2D(:,i,t) = obs(:,i);
%        end
        clearvars obs;
    end
    clearvars obs2D;
%    p.obs2D = reshape(p.obs2D, gN2D);
    p.pdim = getVarFromStruct(cppObj, 'pdim');
    p.pdim = p.pdim+1;
    p.replan = getVarFromStruct(cppObj, 'replan');
    p.target = getVarFromStruct(cppObj, 'target');
    p.target = reshape(p.target, gN);
    p.targetCenter = getVarFromStruct(cppObj, 'targetCenter');
    p.targetR = getVarFromStruct(cppObj, 'targetR');
    p.targetRsmall = getVarFromStruct(cppObj, 'targetRsmall');
    p.targetsm = getVarFromStruct(cppObj, 'targetsm');
    p.targetsm = reshape(p.targetsm, gN);
    p.v = getVarFromStruct(cppObj, 'v');
    p.vReserved = getVarFromStruct(cppObj, 'vReserved');
    p.wReserved = getVarFromStruct(cppObj, 'wReserved');
    xhist = getVarFromStruct(cppObj, 'xhist');
    for t=1:length(xhist)
        p.xhist(:,t) = xhist{t};
    end
end
