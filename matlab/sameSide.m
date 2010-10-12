%% Function sameSide
%
% *Description:* sameSide Technique for working out if points a and b are
% on the same side of the line p1->p2 fast matlab implementation of code
% initially from  http://www.blackpawn.com/texts/pointinpoly/default.html

function Pin=sameSide(p1,p2,a,b)
%% Do this code instead
BminusA=[b-a]';
P1minusA=[p1-a]';
%this is the formula for cross product
cp1 =[BminusA(2,:).*P1minusA(3,:)-BminusA(3,:).*P1minusA(2,:),...
      BminusA(3,:).*P1minusA(1,:)-BminusA(1,:).*P1minusA(3,:),...
      BminusA(1,:).*P1minusA(2,:)-BminusA(2,:).*P1minusA(1,:)];
 

P2minusA=[p2-a]';
cp2 =[BminusA(2,:).*P2minusA(3,:)-BminusA(3,:).*P2minusA(2,:),
      BminusA(3,:).*P2minusA(1,:)-BminusA(1,:).*P2minusA(3,:),
      BminusA(1,:).*P2minusA(2,:)-BminusA(2,:).*P2minusA(1,:)];
%this is quicker than dot product  
if sum(cp1'.*cp2)>=0
    Pin=1;
else
    Pin=0;
end