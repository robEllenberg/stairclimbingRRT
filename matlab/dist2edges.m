%% Function dist2edges
%
% *Description:* Given a starting point (P1) and an end point (P2) of a line, plus another
% point (P3), this function will find the closest point along the line
% (PInt) to P3 and also return the distance (Dist2PInt) between PInt and P3.

function [PIntE,d2edge]=dist2edges(P1,P2,P3) % P1=Node, P2=Parent, P3=Random

% Make sure there is some distance between P1 and P2 (the square of dist between)
distbtwnAllNodes=(P2(:,1)-P1(:,1)).^2 + (P2(:,2)-P1(:,2)).^2 + (P2(:,3)-P1(:,3)).^2;

% Check places with no distance between P1 and P2
newindex=(distbtwnAllNodes==0);

% Need the index so make this approximately zero
distbtwnAllNodes(newindex)=eps;

% Coordinates of closest point
u=((P3(:,1)-P1(:,1)).*(P2(:,1)-P1(:,1))+(P3(:,2)-P1(:,2)).*(P2(:,2)-P1(:,2))+(P3(:,3)-P1(:,3)).*(P2(:,3)-P1(:,3)))./distbtwnAllNodes;
PIntE=[P1(:,1)+u.*(P2(:,1)-P1(:,1)), P1(:,2)+u.*(P2(:,2)-P1(:,2)), P1(:,3)+u.*(P2(:,3)-P1(:,3))];

% If the point of intersection is not between the other two points in either
% or the dist between them was zero, make the point to be at infinity
PIntE((PIntE(:,1)-P1(:,1)).^2+(PIntE(:,2)-P1(:,2)).^2+(PIntE(:,3)-P1(:,3)).^2>(P2(:,1)-P1(:,1)).^2+(P2(:,2)-P1(:,2)).^2+(P2(:,3)-P1(:,3)).^2|...
      (PIntE(:,1)-P2(:,1)).^2+(PIntE(:,2)-P2(:,2)).^2+(PIntE(:,3)-P2(:,3)).^2>(P2(:,1)-P1(:,1)).^2+(P2(:,2)-P1(:,2)).^2+(P2(:,3)-P1(:,3)).^2|...
       newindex,:)=inf;
  
% Calculate the distance to all nodes where PIntE is not infinite
d2edge=zeros([size(PIntE,1),1]);
d2edge(PIntE(:,1)==inf)=inf;
d2edge(PIntE(:,1)~=inf)=sqrt((PIntE(PIntE(:,1)~=inf,1)-P3(:,1)).^2+(PIntE(PIntE(:,1)~=inf,2)-P3(:,2)).^2 + (PIntE(PIntE(:,1)~=inf,3)-P3(:,3)).^2);
