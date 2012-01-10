    clear all;
    close all;

    orEnvLoadScene('openMiniHubo/miniHubo.robot.xml',1);
    robotid = orEnvGetBody('miniHubo');

    %Load any miscellaneous models
    objid = orEnvCreateKinBody('liftbox','box.kinbody2.xml');

    %set printing and display options
    orEnvSetOptions('debug 3')
    orEnvSetOptions('collision bullet')

    manips = orRobotGetManipulators(robotid);
    jointdofs = 0:orRobotGetActiveDOF(robotid);
    %This should be left and right arms...check with robot def.
    activedofs = [manips{1}.armjoints,manips{2}.armjoints];

    %set initial configuration
    initDOFValues2 = [0 0 0 0 0 0 0 0 ];  
    orRobotSetDOFValues(robotid,initDOFValues2,activedofs);
    row4 = [ 0 0 0 1];
    Tbox=[eye(3),[0.00;0.02;0.11];row4];
    orBodySetTransform(objid,Tbox(1:3,4));

    %fid = load('step1traj.txt');

      raw=importdata('step1traj.txt',' ',3);
      graspTraj=raw.data(:,[3:6,10:13]);

     pause(1);

    for i =1:size(graspTraj,1)
          %DOFValues2 = [fid(i,3),1.0*fid(i,4)-pi/7,-1.0*fid(i,5),fid(i,6)-pi/18,fid(i,10),1.0*fid(i,11)-pi/7,-1.0*fid(i,12),fid(i,13)-pi/18 ]; 
          DOFValues2 = graspTraj(i,:);
          DOFValues2(1) = DOFValues2(1);
          DOFValues2(2) = DOFValues2(2)-pi/5;
          DOFValues2(3) = -1.0*DOFValues2(3);
          DOFValues2(4) = DOFValues2(4)-pi/18;
          DOFValues2(5) = DOFValues2(5);
          DOFValues2(6) = DOFValues2(6)-pi/5;
          DOFValues2(7) = -1.0*DOFValues2(7);
          DOFValues2(8) = DOFValues2(8)-pi/18;
          
          orRobotSetDOFValues(robotid,DOFValues2,activedofs);
    pause(0.1);
    end

    [collision,colbodyid,contacts]=orEnvCheckCollision(robotid)

