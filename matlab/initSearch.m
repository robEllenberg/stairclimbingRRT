%% Function initSearch
%
% *Description:* This function plots and outputs some info

function initSearch(Iterations,lim,start,goal,treesMax,dodraw)

global obs;

% Initialise display
% ------------------
if dodraw

    % Output to command window
    fprintf('\n******************************************\n');
    fprintf('***   Rapidly-Exploring Random Trees   ***\n');
    fprintf('******************************************\n\n');
    fprintf('Max. number of steps: %d \n',Iterations);
    fprintf('Max. number of trees: %d \n\n',treesMax);
    
    % Output to figure
    figure(1);
    title('Rapidly-Exploring Random Trees (Step 1)');
    %xlabel('X'); ylabel('Y'); zlabel('Z');
    set(gca,'xtick',[],'ytick',[],'ztick',[]);
    %axis off;
    axis([lim(1,1) lim(1,2) lim(2,1) lim(2,2) lim(3,1) lim(3,2)],'square');
    hold on;

    % Plot initial node
    plot3(start(1),start(2),start(3),'.k');

    % Plot goal node
    plot3(goal(1),goal(2),goal(3),'.b');
    
    % Plot obstacles
    if size(obs,1)>0
        for i=1:size(obs,3)
            fill3([obs(1,1,i) obs(2,1,i) obs(3,1,i) obs(4,1,i) obs(1,1,i)],[obs(1,2,i) obs(2,2,i) obs(3,2,i) obs(4,2,i) obs(1,2,i)],[obs(1,3,i) obs(2,3,i) obs(3,3,i) obs(4,3,i) obs(1,3,i)],'b','EdgeAlpha',0);
            alpha(0.1);
        end
    end
end
