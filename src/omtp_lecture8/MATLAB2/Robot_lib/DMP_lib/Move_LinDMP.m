function [Trj] = Move_LinDMP(Start_pos,Goal_pos)

%%SetupDMP parameters
DMP.dt=dt;
DMP.a_z= 48;
DMP.a_x= 2;
DMP.y0=Start_pos;
DMP.goal=Goal_pos;
DMP.tau=5;
DMP.dy0=[0 0 0];

%%Calculate final phase for the DMP integration (normalized to 500 samples)
NumSamp= 500;
T_f = (NumSamp-1)*DMP.dt;
Xmin = exp(-DMP.a_x*T_f/DMP.tau);

%init. states for position dmp
Sp.y = DMP.y0;
Sp.z = zeros(1,7);
Sp.x = 1;

%%
i=1;
while Sp.x > Xmin
    % position DMP
    [Sp]=DMP_integrate(DMP,Sp,0);
    xN(i,:) = Sp.y;
    
    i=i+1;
end
Trj=xN;
end