%% Function RRT
%
% *Description:* MAIN FUCNTION: This function is a basic implementation of
% the multi-tree Rapidly-exploring Random Tree (RRT) search algorithm with
% the ability to  place discretely possitioned seeds throughout the
% environment which may of may not take root as trees
%
% *Authors:* Gavin Paul & Matthew Clifton
%
% *Last Updated:* 16th September 2008
%
% *Features:*
%   - Multiple trees
%   - Discrete seeding environmental coverage
%   - 2D or 3D search space
%   - Obstacle avoidance
%   - Auto-connect to goal
%   - Path smoothing
%
% *To do:*
%  - Adaptive sampling
%  - Approximate nearest neighbour search

%% Function Call
%
% *Inputs:* 
%
% mt = How many multiple trees (must be at least 2, 1 for source and 1 for destination
%
% nseeds = Number of seeds allowed on each axis (discretely placed seeds which idealy helps the RRT expansion)
%
% n_walls = the Number of walls to be placed in the environment
%
% *Returns:* 
%
% cur_it = How many RRT search itterations needed to find path

function [cur_it]=RRT(mt,nseeds,nwalls)

clear global obs rrt;
global obs rrt;

close all;

%% Check inputs
if nargin<3
    nwalls=3;
    if nargin<2
        nseeds=3;
        if nargin<1
            mt=30;
        end
    end
end

% max trees (mt) needs to be at least 2 trees 1 start and 1 dest
if mt<2; error('You must have at least 2 trees, 1 for start and 1 for dest');end
% nseeds^3 + 2 must be less than or equal to the max trees since they are
% per axis
while nseeds^3+2>mt
    nseeds=nseeds-1;
    display(['Reducing num seeds on each dimension to ',num2str(nseeds)]);
end



%% RUN OPTIONS
%  ************************************************************************
% Display                           => Activate visual display, SIGNIFICANTLY SLOWER
dodraw=true; 

% Obstacle Avoidance                => Include obstacles at all
obstacles_avoid=true;

% Obstacle definitions              => Use objects.txt object definition or default
use_objects_txt=false;

% Path Smoothing                    => Shorten final path
smooth_path=true;

% Step through                   	=> User can step-through program
step_through=false;



%% VARIABLES
%  ************************************************************************

% Iterations                        => Maximum number of search iterations
Iterations=20000;

%objective
objective=0;

% Maximum number of search trees    => Maximum number of search iterations
treesMax=mt;

% Search Space Limits               => [x(min) x(max) y(min) y(max) z(min) z(max)]
% lim=[-0.5 +0.5;-0.5 +0.5 ;-0.5 +0.5];
lim=[-0.5 +0.5;-1 +1 ;-0.5 +0.5];

% Start and Goal Initialisation     => Point [x y z]
% start=[+0.00 -0.45 +0.00];
% goal =[+0.00 +0.45 +0.00];
start=[+0.00 -0.9 +0.00];
goal =[+0.00 +0.9 +0.00];

% Number of attempts at path smoothing
nsmooth=1000;

%% inialise RRT data structures
for i=1:treesMax
    rrt(i).valid=0;
    rrt(i).cords=[0,0,0];
    rrt(i).parent=0;
end

%setup the first tree and first node to be the start and finish
rrt(1).valid=1;
rrt(1).cords=start;
rrt(1).parent=0;

rrt(2).valid=1;
rrt(2).cords=goal;
rrt(2).parent=0;

%if obstacle avoidance is needed then define obstacles
if obstacles_avoid
    obs=obstacles_array(use_objects_txt,nwalls,lim);
end

%% plant the seeds discretely in the environment
seed_cords=[];
for i=1:nseeds
    for j=1:nseeds
        for k=1:nseeds
            seed_cords=[seed_cords;lim(1,1)+i*(lim(1,2)-lim(1,1))/(nseeds+1),...
                                   lim(2,1)+j*(lim(2,2)-lim(2,1))/(nseeds+1),...
                                   lim(3,1)+k*(lim(3,2)-lim(3,1))/(nseeds+1)];
        end
    end
end
%Assign plant seeds a RRT tree to either grow or joint with other trees
for i=3:nseeds^3
    rrt(i).valid=1;
    rrt(i).cords=seed_cords(i,:);
    rrt(i).parent=0;
end

%% MAIN RRT SEARCH ALGORITHM
% *************************************************************************

tic;

% Initial plotting and environment setup
initSearch(Iterations,lim,start,goal,treesMax,dodraw)

% Continue search while the number of steps is less than Iterations and
% treesMax, and a path has not been found
for cur_it=1:Iterations

    % GENERATE A NEW POINT
    new_pnt=newPoint(lim); %if dodraw; plot3(new_pnt(1),new_pnt(2),new_pnt(3),'.c'); end

    % FIND NEAREST NEIGHBOURS    
    [d2nodes,d2edges]=nearestNeighbour(new_pnt);

    % CONNECT TO NEAREST NEIGHBOUR
    [objective]=connect(d2nodes,d2edges,new_pnt,start,goal,dodraw,cur_it,treesMax);
    
    %check if we can break out yet
    if objective; break; end    
    
    %should we do one point at a time
    if step_through; %pause;
        keyboard; end; %uiwait(msgbox('click to continue'));end
end

toc;

% PATH OUTPUT
path=tracePath(dodraw,objective,goal);

% PATH SMOOTHING
if smooth_path && objective
    display('Doing path smoothing now');
    tic
    smoothPath(nsmooth,dodraw,path);
    toc;
end
