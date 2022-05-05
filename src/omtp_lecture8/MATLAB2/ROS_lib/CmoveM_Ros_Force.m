%==========================================================================
function CmoveM_Ros_Force(Td,Fd,time,robot)
% global panda
%==========================================================================
% Calculates cartesian trajectory Ti
% Calculates joint trajectory qi
% Executes joint motion qi
% parameters
%   Td - desired Carthesian pose in world coordinates
%   t - duration


%prepare cartesian trajectory

data = get_Panda_data(robot);
q0 = data.Arm.Actual.Positions;

% T0 = double(panda.fkine(q0));%in robot base
[x,R]=fkin_Panda(q0);

T0=MakeT(R,x);
disp('cmove prepare..')

N = round(time/robot.dt);
Ti = ctraj(T0,Td,N);
%% get updated position of the robot in Joint coordinates 
q=q0;

%prepare joint trj
qi = zeros(size(Ti,3),7);
for i = 1:size(Ti,3)
    x = Ti(1:3,4,i);
    R = Ti(1:3,1:3,i);
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
