%% Function newPoint
%
% *Description:* This function randomly samples the search space. and produces a new node

function new_pnt=newPoint(lim)

% Size of search space
range=abs(lim(:,2)-lim(:,1));

% Randomly generate point
new_pnt=[lim(1,1)+range(1)*rand,...
         lim(2,1)+range(2)*rand,...
         lim(3,1)+range(3)*rand];
