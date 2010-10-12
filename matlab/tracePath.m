%% Function tracePath
%
% *Description:* This function traces the path back through the tree.

function path=tracePath(dodraw,objective,goal)

global rrt;
% Show final result
if  objective
    if dodraw; fprintf('Found a path to the goal!\n\n'); end;
    %it will be in the first tree
    goalnode=find(rrt(1).cords(:,1)==goal(1) & ...
                  rrt(1).cords(:,2)==goal(2) & ...
                  rrt(1).cords(:,3)==goal(3));
    % Trace path back through tree
    path=goal;
%     parent=n;
    while rrt(1).parent(goalnode)~=0
        parent=rrt(1).parent(goalnode);
        path=[rrt(1).cords(parent,:);path];
        goalnode=parent;
    end
    % Plot path
    if dodraw; 
        %delete all other crap off screen
        for t=1:size(rrt,2)
           for i=2:size(rrt(t).parent,1)
               try 
                   delete(plothandles(t).lines(i));
               end 
           end
           try 
               delete(plothandles(t).points);
           end
        end
        plot3(path(:,1),path(:,2),path(:,3),'LineWidth',2,'Color','r'); 
    end
else
    if dodraw; fprintf('Failed to find a path to the goal.\n\n'); end;
    path=[];
end