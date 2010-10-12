%% Function smoothPath
%
% *Description:* Shortens a path by repeatedly attempting to connect two 
% randomly selected points along the path together.

function smoothPath(nsmooth,dodraw,path)
global obs
final_path=path;

% Perform path smoothing
for i=1:nsmooth
    % Randomly select two path segments
    path_length=size(final_path,1);
    p1=ceil((path_length-1)*rand);
    p2=ceil((path_length-1)*rand);
    while (p2==p1); p2=ceil((path_length-1)*rand); end;
    pt=p1; if p1>p2; p1=p2; p2=pt; end;

    % Randomly select two points from the two path segments
    r1=rand;
    if final_path(p1,1)>final_path(p1+1,1); pnt1(1)=final_path(p1,1)-abs(final_path(p1,1)-final_path(p1+1,1))*r1; else pnt1(1)=final_path(p1,1)+abs(final_path(p1,1)-final_path(p1+1,1))*r1; end
    if final_path(p1,2)>final_path(p1+1,2); pnt1(2)=final_path(p1,2)-abs(final_path(p1,2)-final_path(p1+1,2))*r1; else pnt1(2)=final_path(p1,2)+abs(final_path(p1,2)-final_path(p1+1,2))*r1; end
    if final_path(p1,3)>final_path(p1+1,3); pnt1(3)=final_path(p1,3)-abs(final_path(p1,3)-final_path(p1+1,3))*r1; else pnt1(3)=final_path(p1,3)+abs(final_path(p1,3)-final_path(p1+1,3))*r1; end
    r2=rand;
    if final_path(p2,1)>final_path(p2+1,1); pnt2(1)=final_path(p2,1)-abs(final_path(p2,1)-final_path(p2+1,1))*r2; else pnt2(1)=final_path(p2,1)+abs(final_path(p2,1)-final_path(p2+1,1))*r2; end
    if final_path(p2,2)>final_path(p2+1,2); pnt2(2)=final_path(p2,2)-abs(final_path(p2,2)-final_path(p2+1,2))*r2; else pnt2(2)=final_path(p2,2)+abs(final_path(p2,2)-final_path(p2+1,2))*r2; end
    if final_path(p2,3)>final_path(p2+1,3); pnt2(3)=final_path(p2,3)-abs(final_path(p2,3)-final_path(p2+1,3))*r2; else pnt2(3)=final_path(p2,3)+abs(final_path(p2,3)-final_path(p2+1,3))*r2; end

    % Connect the two points
    if ~collisionCheck(pnt1,pnt2,obs);
        % Update Path
        path1=final_path(1:p1,:);
        path2=[pnt1;pnt2];
        path3=final_path(p2+1:end,:);
        final_path=[path1;path2;path3];
        % Plot path
        %if dodraw; line(final_path(:,1),final_path(:,2),final_path(:,3),'Color','y'); end;
    end
end

% Plot final path
if dodraw
    plot3(final_path(:,1),final_path(:,2),final_path(:,3),'LineWidth',2,'Color','g');
    axisH=get(gca,'title');
    title_string=get(axisH,'String');
    title_string=[title_string,'. Initial(Red), Smoothed(Green)'];
    set(axisH,'String',title_string)   
end
