% Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
%   Author: Dmitry Berenson <dberenso@cs.cmu.edu>
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions are met:
%
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of Intel Corporation nor Carnegie Mellon University,
%       nor the names of their contributors, may be used to endorse or
%       promote products derived from this software without specific prior
%       written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
%   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
%   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
%   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
%   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
%   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
%   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
%   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

clear all;
%(constraints on closed-chain kinematics and keeping the box upright

%% Load the environment and setup id's and such. The envload script should bypass this if the envrionment is already loaded.
miniboxlift_envload


% Calculate appropriate transformation from wrist to palm surface, verified
% this works by lining up dummy palms against the box.

%{
rrx = [ 1     0    0;
        0     cosd()   -sind();
        0     sind()    cosd()];
%}
rry180 = [ cosd(180)     0    sind(180);
        0     1    0;
       -sind(180)     0    cosd(180)];

rrz90 = [ cosd(90)    -sind(90)    0;
        sind(90)     cosd(90)    0;
        0     0    1];

rrzn90 = [ cosd(-90)    -sind(-90)    0;
        sind(-90)     cosd(-90)    0;
        0     0    1];

rotation_rightarm = rry180*rrz90;
rotation_leftarm = rry180*rrzn90;

%rotation_rightarm = rrz90*rry180;
%rotation_leftarm =  rrzn90*rry180;

row4=[0 0 0 1];

Rz=[-1 0 0;
    0 -1 0;
    0 0 1];
Tz=[Rz,[0;0;0];row4];


%%%%%%%%%%
%{
RPalm=[ 0.500004	-0.499996	-0.707107;
        0.500004	-0.499996	0.707107;
        -0.707102	-0.707112	-0.000000];
tpalm=[-7.351360;
    -7.332236;
    -57.994285]/1000;

TRPalm=[RPalm,tpalm;row4];


tLpalm=[7.326360;
    -7.481668;
    -58.029640]/1000;
RLPalm=[-0.500004	-0.499996	0.707107;
0.500004	0.499996	0.707107;
-0.707102	0.707112	-0.000000];

TLPalm = [RLPalm,tLpalm;row4];

Tedge=[eye(3),[0;.075;0];row4];
Tbox=[eye(3),[0.27;0;-0.10];row4];
%}

RPalm=[ 1	0	0;
        0	1	0;
        0	0	1];
tpalm=[0;
    -0.024;
    0];

rr = eye(3);

%TRPalm=[RPalm,tpalm;row4];
TRPalm=[(rotation_rightarm),tpalm;row4];

tLpalm=[0;
    -0.024;
    0];

RLPalm=[1	0	0;
        0	1	0;
        0	0	1];

%TLPalm = [RLPalm,tLpalm;row4];
TLPalm = [(rotation_leftarm),tLpalm;row4];



rr = eye(3);
%Tedge=[rr,[0.0478;0;0];row4];
Tedge=[rr,[0.0478;0;0];row4];


Tbox=[eye(3),[0.00;0.02;0.11];row4];


TInitRight=Tedge^-1*Tbox*TRPalm^-1;
TInitLeft=Tedge*Tbox*TLPalm^-1;

%TInitRight=[(rotation_rightarm),[0;0;0];row4] * TInitRight;
%TInitLeft =[(rotation_leftarm),[0;0;0];row4]  * TInitLeft;

%?%
%{
transoffset0 = [-0.04781 0 0.00];
transoffset1 = [0.04781 0 0.00];
handtrans0 = Tbox(1:3,4)' + transoffset0;
handrot0 = rodrigues([0 0 0]);
handtrans1 = Tbox(1:3,4)' + transoffset1;
handrot1 = rodrigues([0 0 0]);
TInitRight = MakeTransform(handrot0,handtrans0');
TInitLeft = MakeTransform(handrot1,handtrans1');
%}


%Define the lift motion relative to current pose
%%%%%%%%%%Tlift=[eye(3),[0.0;0.0;0.2];row4];
Tlift=[eye(3),[0.0;0.05;0];row4];

%Inverse rotation to align the palm with the side of the box.

orBodySetTransform(objid,Tbox(1:3,4));


%create the problem instances we need
%%%%%%%%%%probs.manip = orEnvCreateProblem('Manipulation','jaemiHubo');
%%%%%%%%%%probs.cbirrt = orEnvCreateProblem('CBiRRT','jaemiHubo');
probs.manip = orEnvCreateProblem('Manipulation','miniHubo');
probs.cbirrt = orEnvCreateProblem('CBiRRT','miniHubo');

%get the descriptions of the robot's manipulators
manips = orRobotGetManipulators(robotid);

%define which joints are active
jointdofs = 0:orRobotGetActiveDOF(robotid);
%This should be left and right arms...check with robot def.
activedofs = [manips{1}.armjoints,manips{2}.armjoints];


%set initial configuration
%initIKPose = [0.000 pi/2 0.000 0.0000 0.0000 0.0000 0.0000 0.000 pi/2 0 0 0 0 0];  
%initDOFValues = [0.000 0.500 0.000 0.0000 0.0000 0.0000 0.0000 0.000 0.500 0 0 0 0 0];  
initIKPose2 =     [0 pi/2 0 0  0 pi/2 0 0 ];  
%initIKPose2 =     [pi/2 0 pi/2 pi/2 pi/2 0 pi/2 pi/2 ];  
initDOFValues2 =  [0 0 0 0     0 0 0 0 ];   
orRobotSetDOFValues(robotid,initIKPose2,activedofs);

%{
orRobotSetDOFValues(robotid,0,activedofs(1));
orRobotSetDOFValues(robotid,0,activedofs(2));
orRobotSetDOFValues(robotid,0,activedofs(3));
orRobotSetDOFValues(robotid,0,activedofs(4));
orRobotSetDOFValues(robotid,0,activedofs(5));
orRobotSetDOFValues(robotid,0,activedofs(6));
orRobotSetDOFValues(robotid,0,activedofs(7));
orRobotSetDOFValues(robotid,0,activedofs(8));
%}

disp('Initial Pose Set, press any Key to continue...');
pause(1);



%get the ik solutions for both arms for the box in the start pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
%%%%%%%%startik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str(flattenTransMat(TInitRight))],probs.cbirrt);
startik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(TInitRight),GetTrans(TInitRight)])],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
%%%%%%%%startik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str(flattenTransMat(TInitLeft))],probs.cbirrt);
startik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(TInitLeft),GetTrans(TInitLeft)])],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
startik = [startik0 ' ' startik1];
orRobotSetDOFValues(robotid,str2num(startik));

%startik = [    1.5708 0 1.5708 1.5708 1.5708      0  1.5708  1.5708];

disp('Solved first grasping pose, press any Key to continue...');
pause(1)

%{

%% Define initial and goal positions

TGoalLeft=Tlift*TInitLeft;
TGoalRight=Tlift*TInitRight;

%define the target transform of the goal
TBoxGoal=Tlift*Tbox;


orBodySetTransform(objid,flattenTransMat(TBoxGoal));


%get the ik solutions for both arms for the box in the goal pose
orRobotSetActiveDOFs(robotid,manips{1}.armjoints);
%%%%%%%%goalik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str(flattenTransMat(TGoalRight)) ],probs.cbirrt);
goalik0 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str([GetRot(TGoalRight),GetTrans(TGoalRight)]) ],probs.cbirrt);
orRobotSetActiveDOFs(robotid,manips{2}.armjoints);
%%%%%%%%goalik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str(flattenTransMat(TGoalLeft)) ],probs.cbirrt);
goalik1 = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 1 ' num2str([GetRot(TGoalLeft),GetTrans(TGoalLeft)]) ],probs.cbirrt);

orRobotSetActiveDOFs(robotid,activedofs);
goalik = [goalik0 ' ' goalik1];
orRobotSetDOFValues(robotid,str2num(goalik));

disp('Solved final grasping pose, press any Key to continue...');
pause(1);


%reset the box to the start pose
orBodySetTransform(objid,Tbox(1:3,4));


%plan a reaching motion to grab the box
orRobotSetDOFValues(robotid,initIKPose2,activedofs);

orProblemSendCommand(['RunCBiRRT jointgoals '  num2str(numel(str2num(startik))) ' ' num2str(startik) ],probs.cbirrt);

%execute the planned trajectory
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt);
orEnvWait(robotid);

!cp cmovetraj.txt step1traj.txt

disp('Arms in initial pose');
pause(1);


%%%%%%%%%%Tw_e0=Tedge^-1*TRPalm^-1;
%%%%%%%%%%Tw_e1=Tedge*TLPalm^-1;

Tw_e0= MakeTransform(handrot0,transoffset0');
Tw_e1= MakeTransform(handrot1,transoffset1');

%specify a TSR for manipulator 0 to keep the box from tilting during the motion
TSRstring0 = SerializeTSR(0,'NULL',Tbox,Tw_e0,[-1000 1000  -1000 1000  -1000 1000  0 0  -pi pi  0 0])

%specify a TSR for manipulator 1 to keep its end-effector fixed relative to the box
TSRstring1 = SerializeTSR(1,'liftbox body',eye(4),Tw_e1,zeros(1,12))

%pack the TSRs into a string
TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring0,'NULL',[]) ' ' SerializeTSRChain(0,0,1,1,TSRstring1,'NULL',[])]

%grab the box with manipulator 0
%TODO: figure out how to release the object
orProblemSendCommand(['setactivemanip index 0'],probs.manip)
orProblemSendCommand(['GrabBody name liftbox'],probs.manip)

disp('Box Grabbed')
[collision,colbodyid,contacts]=orEnvCheckCollision(robotid)



%% Plan the lifting motion

goals_in = [str2num(goalik)];
orProblemSendCommand(['RunCBiRRT timelimit 20 smoothingitrs 100 jointgoals '  num2str(numel(goals_in)) ' ' num2str(goals_in) ' ' TSRChainString],probs.cbirrt)
[collision,colbodyid,contacts]=orEnvCheckCollision(robotid)

%execute the planned trajectory
orProblemSendCommand(['traj cmovetraj.txt'],probs.cbirrt)

orEnvWait(robotid);

!cp cmovetraj.txt step2traj.txt
%}

