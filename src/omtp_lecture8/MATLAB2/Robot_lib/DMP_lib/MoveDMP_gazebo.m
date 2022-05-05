%%
clc
clear all
close all
dt=1/30;
path('../ROS_lib',path)
path('../DMP_Library',path)
load test_trj.mat

%% TODO: Implement the encoding step for the Joint DMP from the previous
%exercise.
JDMP = LearnJDMP(qJoints,dt);

%%Calculate final phase for the DMP integration
T_f = (length(qJoints)+1)*JDMP.dt;
Xmin = exp(-JDMP.a_x*T_f/JDMP.tau);

%init. states for position dmp
Sj.y = JDMP.y0;
Sj.z = zeros(1,7);
Sj.x = 1;

%% Execute in simulation
rosinit
 try
    %connect to sim robot
    
    [robot,q0] = init_Panda(dt);
    Startjoints = qJoints(1,:);
    
    %% move to init position
    Jmove_Ros(Startjoints,2,robot);
    disp('start DMP')
    pause(3)
    
    i=1;
    tn=0;
    st = tic;
    
    Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    Arm_send.TimeFromStart.Nsec = 330000000;
    
    while Sj.x > Xmin
        
        %% TODO: JDMP integration simmilar as in the previous exercise
        %% Joint DMP
        [Sj]=DMP_integrate(JDMP,Sj,0);
        jN(i,:) = Sj.y;
        jjN(i,:) = Sj.z;
        
        %% Send to robot

        Arm_send.Positions = Sj.y;
        Arm_send.Velocities = Sj.z;
        robot.ArmM.Points= Arm_send;
        send(robot.ArmP,robot.ArmM);
  
        %% sinhronisation
        tn = tn+dt;
        if tn>toc(st)
            pause(tn-toc(st))
        end
        i=i+1;
    end
    
    disp('finished')
    %% move to start position
    Jmove_Ros([0 0 0 -pi/2 0 pi/2 0],2,robot);
    
    pause(3)
    
catch ME
    ErrorTrap(ME);
end

rosshutdown

% Plot Joint trajectories
figure(3)
plot(jN,'r') % plot JDMP trajectory
hold on
plot(qJoints,'--b') % plot example trajectory