function [dir1, dir2] = HJIDirectionCoop4D(g,P,x1,x2)
% [ua, ud] = HJIStrategy(g,P,xa,xd)
% Calculates the optimal control in the HJI sense
% 
% Inputs:
%   g       - 4D grid structure
%   P       - array of gradients of the reachable set
%   xa, xd  - attacker and defender positions (size must be [1 2])
%
% Outputs
%   ua, ud  - unit row vectors representing control input direction
% 
% Mo Chen, Oct. 7, 2013
%

p = calculateCostate(g,P,[x1 x2]);

small = 1e-2;
for i = 1:4
    if abs(p(i)) < small
        p(i) = 0;
    end
end

p1 = p(1:2);
p2 = p(3:4);

if norm(p1)>small,  dir1 = -p1 / norm(p1);
else                dir1 = [0 0]; end

if norm(p2)>small,  dir2 = -p2 / norm(p2); 
else                dir2 = [0 0]; end

end