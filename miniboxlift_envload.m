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

%An example of using TSRs for lifting a box with HERB2 
%(constraints on closed-chain kinematics and keeping the box upright
if ~exist('BOXLIFT_LOADED')
    clear all;
    close all;

    %load the robot into the environment
    %%%%%%orEnvLoadScene('openHubo/jaemiHubo.robot.xml',1);
    %%%%%%robotid = orEnvGetBody('jaemiHubo');

    orEnvLoadScene('openMiniHubo/miniHubo.robot.xml',1);
    robotid = orEnvGetBody('miniHubo');

    %Load any miscellaneous models
    objid = orEnvCreateKinBody('liftbox','openHubo/box.kinbody.xml');
    %rightPalmID = orEnvCreateKinBody('rightPalm','../../hubo/openHubo/rightPalm.kinbody.xml');
    %leftPalmID = orEnvCreateKinBody('leftPalm','../../hubo/openHubo/leftPalm.kinbody.xml');

    %set printing and display options
    orEnvSetOptions('debug 3')
    orEnvSetOptions('collision bullet')

    manips = orRobotGetManipulators(robotid);
    jointdofs = 0:orRobotGetActiveDOF(robotid);
    %This should be left and right arms...check with robot def.
    activedofs = [manips{1}.armjoints,manips{2}.armjoints];

    %set initial configuration
    %initDOFValues = [0.000 0.500 0.000 0.0000 0.0000 0.0000 0.0000 0.000 0.500 0 0 0 0 0];  
    %orRobotSetActiveDOFs(robotid,activedofs);

    initDOFValues2 = [0 0 0 0 0 0 0 0 ];  
    orRobotSetDOFValues(robotid,initDOFValues2,activedofs);
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

    BOXLIFT_LOADED=1;
    disp('Box lifting environment with Jaemi Hubo loaded')

else
    disp('Environment already loaded, clear BOXLIFT_LOADED to reload')
end

