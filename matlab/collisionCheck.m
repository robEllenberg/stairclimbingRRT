%% Function collisionCheck
% 
% *Description:* Returns whether a collision occurs between an edge and a set
% of obstacles. Also gives the point of intersection. (P1=node,P2=parent node)

function [collision,PInt]=collisionCheck(P1,P2,obs)

global collcount;
collcount=collcount+1;
%default is that it is safe, then if a collision is found it is set to 1
%and we return
collision=0;

if size(obs,1)==0
    PInt=inf;
    return;
end
% Calculate intercept point of a line and plane
% ---------------------------------------------

% Line equation
r_var=[P1(1)-P2(1) P1(2)-P2(2) P1(3)-P2(3)];

for i=1:size(obs,3)

    % Normal vector (this code is quicker than 'cross(obs2-obs1,obs1-obs3)')
    a=[obs(2,:,i)-obs(1,:,i)]'; b=[obs(1,:,i)-obs(3,:,i)]';
    normalVec=[a(2,:).*b(3,:)-a(3,:).*b(2,:);a(3,:).*b(1,:)-a(1,:).*b(3,:);a(1,:).*b(2,:)-a(2,:).*b(1,:)]';

    % Plane equation
    plane_equ=[normalVec(1:3),-sum(obs(1,:,i).*normalVec)];

    % Plane * Line
    bottomof_t_var=plane_equ(1)*r_var(1)+plane_equ(2)*r_var(2)+plane_equ(3)*r_var(3);

    % Some variable
    t_var=(plane_equ(1)*P1(1)+plane_equ(2)*P1(2)+plane_equ(3)*P1(3)+plane_equ(4))./bottomof_t_var;

    % Get intersection points
    PInt=[t_var.*-r_var(1)+P1(1),...
          t_var.*-r_var(2)+P1(2),...
          t_var.*-r_var(3)+P1(3)];
%     plot3(PInt(1),PInt(2),PInt(3),'r*');

    % Check if intercept point lies within line segment
    sqrd_distbetweenpnts=(P2(1)-P1(1))^2+(P2(2)-P1(2))^2+(P2(3)-P1(3))^2;
    if (PInt(1)-P1(1))^2+(PInt(2)-P1(2))^2+(PInt(3)-P1(3))^2 < sqrd_distbetweenpnts && ...
       (PInt(1)-P2(1))^2+(PInt(2)-P2(2))^2+(PInt(3)-P2(3))^2 < sqrd_distbetweenpnts;
        % Test whether intercept point is within boundaries
        % -------------------------------------------------
        % Check if point lies within plane boundaries
        % Ref: http://www.blackpawn.com/texts/pointinpoly/default.html
        if PointInQuad(PInt,obs(:,:,i));
            collision=1;
            %no need to check anymore since there is a collision 
            return;
        end        
    end
end
