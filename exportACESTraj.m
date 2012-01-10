%% post-process trajectories generated with CBiRRT
function [fname,data]=exportACESTraj(name,tstep,tTotal,fname,joints)
    
    fid=fopen(name);

    if ~exist('fname')
        fname=['new-' name];
    end

    line1=fgetl(fid);
    line2=fgetl(fid);
    initdata=sscanf(line2,'%f')';

    A=[];
    while ~feof(fid)
        tline=fgets(fid);
        A=[A;sscanf(tline,'%f')'];
    end
    fclose(fid);

    t0=-2;
    rawTimes=A(:,1);
    tmax=max(rawTimes);
    timeVec=[t0;rawTimes*tTotal/tmax];
    
    tused=[t0:tstep:tTotal];
    
    data=interp1(timeVec,[zeros(1,size(A,2)-1);A(:,2:end)],tused);


    fout=fopen(fname,'w');
    fprintf(fout,'%s ',joints.names{:});
    fprintf(fout,'\n');

    for k=1:length(tused)
        %compensate for zero indexing of joint names
        fprintf(fout,'%f ',data(k,joints.indexVector+1).*joints.signVector);
        fprintf(fout,'\n');
    end

    fclose(fout);
end

