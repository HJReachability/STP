function RTTRS = loadRTTRS(filename)
  load(filename);
  RTTRS.g = cpp2matG(RTTRS.g);
  RTTRS.dynSys = PlaneCAvoid(...
      zeros(3,1), ...
      RTTRS.dynSys.wMaxA, RTTRS.dynSys.vRangeA, ... 
      RTTRS.dynSys.wMaxB, RTTRS.dynSys.vRangeB, ...
      RTTRS.dynSys.dMaxA, RTTRS.dynSys.dMaxB);
  RTTRS.dynSys.hdim = RTTRS.dynSys.hdim+1;
  RTTRS.dynSys.hpv = RTTRS.dynSys.hpv;
  RTTRS.dynSys.hpvhist = RTTRS.dynSys.hpvhist;
  RTTRS.dynSys.nd = RTTRS.dynSys.nd;
  RTTRS.dynSys.nu = RTTRS.dynSys.nu;
  RTTRS.dynSys.nx = RTTRS.dynSys.nx;
  RTTRS.dynSys.pdim = RTTRS.dynSys.pdim+1;
  RTTRS.dynSys.x = RTTRS.dynSys.x;
  RTTRS.dynSys.xhist = RTTRS.dynSys.xhist;
end