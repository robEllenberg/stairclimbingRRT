%% Function PointInQuad
%
% *Description:* Is point of intersection inside the obstacle uses sameSide
% usually used for checking a point in a mesh (triangle) but here we do the
% same for a 4 point plannar object   

function test_plane=PointInQuad(Pint,obs)
%% this is slightly faster
test_plane=0;
if sameSide(Pint,obs(3,:,:),obs(1,:,:),obs(2,:,:)) 
    if sameSide(Pint,obs(4,:,:),obs(2,:,:),obs(3,:,:)) 
        if sameSide(Pint,obs(1,:,:),obs(3,:,:),obs(4,:,:)) 
            if sameSide(Pint,obs(2,:,:),obs(4,:,:),obs(1,:,:))
                test_plane=1;
            end
        end
    end
end

