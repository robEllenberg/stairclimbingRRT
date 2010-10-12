%% Function nearestNeighbour
%
% *Description:* This function finds the nearest node or edge.

function [d2nodes,d2edges]=nearestNeighbour(new_pnt)
        
global rrt nncount;
nncount=nncount+1;

%go through each tree and find vector of dist to edge and nodes
for t=1:size(rrt,2)
    if rrt(t).valid
        % Distance to all nodes 
        d2nodes(t).vals=sqrt((rrt(t).cords(:,1)-new_pnt(1)).^2+...
                             (rrt(t).cords(:,2)-new_pnt(2)).^2+...
                             (rrt(t).cords(:,3)-new_pnt(3)).^2);
        
        % Distance to all edges (if there are any)
        if size(rrt(t).cords,1)>1
            lineStarts=rrt(t).cords(2:end,:);
            lineEnds=rrt(t).cords(rrt(t).parent(2:end),:);            
            
            [d2edges(t).node,d2edges(t).vals]=dist2edges(lineStarts,lineEnds,new_pnt);
        else
            d2edges(t).node=[inf,inf,inf];d2edges(t).vals=[inf];
        end
    end
end

