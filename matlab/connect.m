%% Function connect
% 
% *Description:* This function attempts to connect new_pnt to the nearest node or edge of 
% each tree.

function [objective]=connect(d2nodes,d2edges,new_pnt,start,goal,do_draw,cur_it,treesMax)

global rrt obs;

objective=0;
addedtotree=zeros([size(rrt,2),1]);
%find the valid trees
for t=1:size(rrt,2)
    if rrt(t).valid==1

        %get the min values of the nodes and edges
        [unused_val,minNode_index]=min(d2nodes(t).vals);
        [unused_val,minEdge_index]=min(d2edges(t).vals);

        %determine the closest (node or edge)
        if d2nodes(t).vals(minNode_index)<=d2edges(t).vals(minEdge_index) %then try and get to the node first
            % Check for collision  
            if ~collisionCheck(new_pnt,rrt(t).cords(minNode_index,:),obs);
                rrt(t).parent=[rrt(t).parent;minNode_index];
                rrt(t).cords=[rrt(t).cords;new_pnt];
                addedtotree(t)=1;
            end

        else %try and get to the edge
            % Check for collision 
            if d2edges(t).vals(minEdge_index)<inf && ~collisionCheck(new_pnt,d2edges(t).node(minEdge_index,:),obs);
                %this is really tricky!! because the index
                %of the edges where the first point is the start of
                %the line, this will also be the parent of that
                %middle node
                rrt(t).parent=[rrt(t).parent;rrt(t).parent(minEdge_index+1)];
                rrt(t).cords=[rrt(t).cords;d2edges(t).node(minEdge_index,:)];
                rrt(t).parent=[rrt(t).parent;size(rrt(t).parent,1)];
                rrt(t).cords=[rrt(t).cords;new_pnt];
                addedtotree(t)=1;
            end
        end        
    end
end

% need to find out if we need to add a tree and put the new point as the
if sum(addedtotree)>1 %then we have connected 2 trees
    trees_added_to=find(addedtotree==1);  
    %the linking node from first trees parent
    parentnode=rrt(trees_added_to(1)).parent(end);
   
        %this bit is tricky TOO since you have to plant the tree by one of
        %its branches and then somehow make the parent correct. The newpoint added to 2
        %different trees currently has 2 (or more) parents

    %go through all other tree which were connected in
    for i=2:size(trees_added_to)
        %store the original parents
        original_parents=rrt(trees_added_to(i)).parent;
        %start at the connecting node (which should be the last node)
        curnode=size(rrt(trees_added_to(i)).parent,1);
        %asign new parent to the other trees connecting node
        rrt(trees_added_to(i)).parent(curnode)=parentnode;
        
        %go through and trace route back to start and swap parents around,
        %all nodes not on this route keep the same parents (shifted)
        while original_parents(curnode)~=0
            thisnodes_parent=original_parents(curnode);
            rrt(trees_added_to(i)).parent(thisnodes_parent)=curnode;
            curnode=thisnodes_parent;
        end
        %concaternate coords into first tree
        rrt(trees_added_to(1)).cords=[rrt(trees_added_to(1)).cords(1:end-1,:);rrt(trees_added_to(i)).cords];
        %concaternate parents together and shift the seconds ones
        rrt(trees_added_to(1)).parent=[rrt(trees_added_to(1)).parent(1:end-1);...
                                       rrt(trees_added_to(i)).parent(1:end-1)+size(rrt(trees_added_to(1)).parent,1)-1;...
                                       parentnode];
       rrt(trees_added_to(i)).valid=0;
    end
    
    %since we have connected 2 trees we may now have a solution
    %test to see if start and goal are in the same tree
    foundstart=find(rrt(trees_added_to(1)).cords(:,1)==start(1) &...
                    rrt(trees_added_to(1)).cords(:,2)==start(2) & ...
                    rrt(trees_added_to(1)).cords(:,3)==start(3));
        
    foundgoal=find(rrt(trees_added_to(1)).cords(:,1)==goal(1) &...
                    rrt(trees_added_to(1)).cords(:,2)==goal(2) & ...
                    rrt(trees_added_to(1)).cords(:,3)==goal(3));
                
    %then the solution has been found
    if ~isempty(foundstart)&&~isempty(foundgoal); objective=1;end

%if we have not added any then we try and make a new tree if we can, otherwise point is lost    
elseif sum(addedtotree)==0
    %couldn't connect to any points or edges within the first tree
    %so make a new tree
    for i=1:size(rrt,2)
        if rrt(i).valid==0
            rrt(i).valid=1;
            rrt(i).cords=new_pnt;
            rrt(i).parent=0;
            break
        end
    end
end
%if we want to draw this shit up
if do_draw
    displayTree(cur_it,treesMax);
end
