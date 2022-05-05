%==========================================================================
function Move_Trj(Trj,R0,robot)
% global panda
%==========================================================================
% Trj Calculated Carthesian Trajectory 
% R0 start constant orientation


%prepare cartesian trajectory

data = get_Panda_data(robot);
q0 = data.Arm.Actual.Positions;

%% get updated position of the robot in Joint coordinates 
q=q0;

%prepare joint trj
qi = zeros(size(Trj,3),7);
for i = 1:size(Ti,3)
    x = Trj(i,:);
    R = R0;
    q = ikin_Panda(x,R,q);
    qi(i,:) = q;
%     disp('.a.')
end
disp('velocities')
%% calculate velocities
qdi = diff(qi)/robot.dt;
qdi = [qdi;qdi(end,:)];

%execute
disp('cmove exec..')

Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Arm_send.TimeFromStart.Nsec = 10000000;
tn=0;
st = tic;

for i = 1:size(qi,1)
    Arm_send.Positions = qi(i,:);
    Arm_send.Velocities = qdi(i,:);
    robot.ArmM.Points= Arm_send;
    send(robot.ArmP,robot.ArmM);
    tn = tn+robot.dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end

disp('done.');
