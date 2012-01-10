%load the robot into the environment
orEnvLoadScene('openHubo/jaemiHubo.rightArm.robot.xml',1);
robotid = orEnvGetBody('jaemiHubo');

%set printing and display options
orEnvSetOptions('debug 4')
orEnvSetOptions('collision ode')

manips = orRobotGetManipulators(robotid);

activedofs = [manips{1}.armjoints];
orRobotSetActiveDOFs(robotid,activedofs);

%create the problem instances we need
probs.cbirrt = orEnvCreateProblem('CBiRRT','jaemiHubo');

%get the descriptions of the robot's manipulators
manips = orRobotGetManipulators(robotid);

%set initial configuration
initDOFValues = [.1 -.5 0 -1 0 0 0];

for k=1:200
    r=rand()*pi-pi/2;
    p=rand()*pi-pi/2;
    y=rand()*pi-pi/2;

    R_t=Rz(y)*Rx(r)*Ry(p);
    t=rand(3,1)*.5 - .5*[.5 .5 .5]';

    T_t=[R_t,t;0 0 0 1];

    %get the ik solutions for both arms for the box in the start pose
    startik = orProblemSendCommand(['DoGeneralIK exec nummanips 1 ' ' maniptm 0 ' num2str(flattenTransMat(T_t))],probs.cbirrt)

    if length(startik)>0
        disp('Found valid solution');

        orRobotSetDOFValues(robotid,str2num(startik));
        pause(1)
        orRobotSetDOFValues(robotid,initDOFValues,activedofs);
        pause(1)
    else
        disp('No solution found!')
    end
end


