
function [DMP] = JointDMP(path,dt,N)
    %set DMP parameters
    if nargin == 3
        DMP.N = N;
    else
        DMP.N = 20;
    end
    DMP.dt=dt;DMP.a_z=48;DMP.a_x=2;
    %learning the joint positions
    DMP=DMP_rlearn(path,DMP);

end