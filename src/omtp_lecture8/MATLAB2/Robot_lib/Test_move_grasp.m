%% move test and grasp

%%
clc
clear all
% global panda
close all
dt=1/30;
path('../ROS_lib',path)
path('DMP_lib',path)


rosinit

%% Robot pick locations
q(1,:)= [-0.0001 -0.7846 -0.0000 -2.3586 -0.0000 1.5697 0.7850];
q(2,:)= [0.7854 -0.7846 -0.0000 -2.3586 -0.0000 1.5697 0.7850];
q(3,:)= [-1.5708 -0.7846 -0.0000 -2.3586 -0.0000 1.5697 0.7850];
%Release
q(4,:)= [-deg2rad(120) -0.7846 0 -2.3586 0 1.5697 0.7850];


%% cube pos
C(1,:)=[0.52, 0, 0.12];
C(2,:)=[0.4, 0.4, 0.12];
C(3,:)=[0, -0.5, 0.12];


[robot] = init_Panda(dt);


%% move to init position
Jmove_Ros(q(1,:),2,robot);
OpenHand(robot);
pause(3)

for i = 1:size(C,2)
    
    Jmove_Ros(q(i,:),2,robot);
    [x0,R0]=fkin_Panda(q(1,:));
    
    
    T= MakeT(R0,C(i,:));
    CmoveM_Ros(T,4,robot)
    
    pause(2)
    CloseHand(robot);
    pause(2)
    
    Jmove_Ros(q(1,:),2,robot)
    
    pause(2)
    Jmove_Ros(q(4,:),2,robot)
    
    OpenHand(robot);
    
    
end
Jmove_Ros(q(1,:),2,robot)

rosshutdown
