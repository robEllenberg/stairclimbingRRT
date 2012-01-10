if ~exist('BOXLIFT_LOADED')
    close all;

    %load the robot into the environment
    orEnvLoadScene('openHubo/jaemiHubo.robot.xml',1);
    robotid = orEnvGetBody('jaemiHubo');

    %Load any miscellaneous models
    objid = orEnvCreateKinBody('liftbox','openHubo/box.kinbody.xml');

    %set printing and display options
    orEnvSetOptions('debug 3')
    orEnvSetOptions('collision ode')

    manips = orRobotGetManipulators(robotid);
    jointdofs = 0:orRobotGetActiveDOF(robotid);
    %This should be left and right arms...check with robot def.
    activedofs = [manips{1}.armjoints,manips{2}.armjoints];

    %set initial configuration

    BOXLIFT_LOADED=1;
    disp('Box lifting environment with Jaemi Hubo loaded')

else
    disp('Environment already loaded, clear BOXLIFT_LOADED to reload')
end

