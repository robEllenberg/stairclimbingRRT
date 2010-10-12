%% Function obstacles_array
%
% *Description:* This either loads the use_objects_txt which is a setup as
% multiple sets of 4 3D point which define a planar obstacle. Or we use the
% default walls where nwalls decides how many walls there are to be

function obs=obstacles_array(use_objects_txt,nwalls,lim)

% Obstacles                     => Define a obstacles using four points [x(1:4,:) y(1:4,:) z(1:4,:)]
if use_objects_txt
    try obs_temp=load('obstacles.txt');
    catch 
        error('Cant load obstacles.txt');
    end    
    num_obs=size(obs_temp,1)/4;
    for i=1:num_obs
        obs(:,:,i)=obs_temp(i*4-3:i*4,:);
    end
    
else %use default obstacle walls
    if nwalls==1
        y=0;
        obs(:,:,1:4)=wall_1(y);
    elseif nwalls>1
        for i=1:nwalls
            y=lim(2,1)+0.2+(i-1)*(lim(2,2)-lim(2,1)-0.4)/(nwalls-1);
            if mod(i,2)==1
                obs(:,:,i*4-3:i*4)=wall_1(y);
            else
                obs(:,:,i*4-3:i*4)=wall_2(y);
            end
        end
    end
end

%% First example wall obstacle with hole
function wall1=wall_1(y)
wall1(:,:,1)=[-0.50 y +0.50; 
              +0.15 y +0.50; 
              +0.15 y -0.50; 
              -0.50 y -0.50];
          
wall1(:,:,2)=[+0.35 y +0.50; 
              +0.50 y +0.50; 
              +0.50 y -0.50; 
              +0.35 y -0.50];
          
wall1(:,:,3)=[+0.15 y +0.50; 
              +0.35 y +0.50; 
              +0.35 y +0.35; 
              +0.15 y +0.35];
          
wall1(:,:,4)=[+0.15 y +0.15; 
              +0.35 y +0.15; 
              +0.35 y -0.50; 
              +0.15 y -0.50];

%% Second wall obstacle with hole in different spot
function wall2=wall_2(y) 
wall2(:,:,1)=[-0.50 y +0.50; 
              -0.35 y +0.50; 
              -0.35 y -0.50; 
              -0.50 y -0.50];
          
wall2(:,:,2)=[-0.15 y +0.50; 
              +0.50 y +0.50; 
              +0.50 y -0.50; 
              -0.15 y -0.50];
          
wall2(:,:,3)=[-0.35 y +0.50; 
              -0.15 y +0.50; 
              -0.15 y -0.15; 
              -0.35 y -0.15];
          
wall2(:,:,4)=[-0.35 y -0.35; 
              -0.15 y -0.35; 
              -0.15 y -0.50; 
              -0.35 y -0.50];
