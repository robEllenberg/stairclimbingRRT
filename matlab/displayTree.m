%% Function displayTree
%
%  *Description:* This function displays the results of the search.

function displayTree(cur_it,treesMax)

global rrt plothandles;
validtrees=0;
colours=['b','k','c','m','y','g'];

for t=1:size(rrt,2)
    if rrt(t).valid
        validtrees=validtrees+1;
        % Plot new edges               
        for i=2:size(rrt(t).parent,1)            
            try delete(plothandles(t).lines(i));end
            plothandles(t).lines(i)=plot3([rrt(t).cords(rrt(t).parent(i),1),rrt(t).cords(i,1)],...
                                          [rrt(t).cords(rrt(t).parent(i),2),rrt(t).cords(i,2)],...
                                          [rrt(t).cords(rrt(t).parent(i),3),rrt(t).cords(i,3)],colours(mod(t,size(colours,2))+1)); %,'Color',[t/(treesMax*3) t/(treesMax*1) t/(treesMax*2)]); %[t/size(rrt,2),t/size(rrt,2),t/size(rrt,2)]);
        end
%         try delete(plothandles(t).points);end
%         plothandles(t).points=plot3(rrt(t).cords(:,1),rrt(t).cords(:,2),rrt(t).cords(:,3),'+','Color',[t/(treesMax*3) t/(treesMax*1) t/(treesMax*2)]);
    else
        for i=2:size(rrt(t).parent,1);  try delete(plothandles(t).lines(i));end; end
        try delete(plothandles(t).points); end
    end
end
% Plot title
title(['Rapidly-Exploring Random Trees (Step: ', num2str(cur_it), '), No. of active trees = ',num2str(validtrees)]);
drawnow;
