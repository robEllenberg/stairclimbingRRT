function visualizeDOF(id,DOFs)

    for j=1:length(DOFs)
        fprintf('Moving Joint %d\n',j)
        for k=[0:.02:.2,.2:-.02:0];
            orRobotSetDOFValues(id,k,DOFs(j))
            pause(.05)
        end
    end
    orEnvWait(.1);
    pause(.1)

end
