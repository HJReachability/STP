clear all; close all;
N = 4;
for vnum = 1:N
%     [ reach, g, tau, obstacle] = coop3D('veryHigh', vnum);
%     save(['coop3D_' num2str(vnum)],'reach','g','tau','obstacle')
    load(['coop3D_' num2str(vnum)],'reach','g','tau','obstacle')
    simulate3D;
%     save(['coop3D_' num2str(vnum)],'traj','-append')
    simulate3Dall;
end