%==========================================================================
function JmoveTrj(qi,robot)
%==========================================================================
% join move from teh current joints to the (q) pose in (t) secconds

disp('jmove trj..')
qi=qi;

%prepare trajectory
qdi = diff(qi)/dt;
[B,A] = butter(2,0.3);
qdif = filtfilt(B,A,qdi);
% be sure to end with 0 velocity
qdif = [qdif; zeros(1,7)];


Arm_send = rosmessage("trajectory_msgs/JointTrajectoryPoint");
Arm_send.TimeFromStart.Nsec = 10000000;
tn=0;
st = tic;

for i = 1:size(qi,1)
    Arm_send.Positions = qi(i,:);
    Arm_send.Velocities = qdif(i,:);
    robot.ArmM.Points= Arm_send;
    send(robot.ArmP,robot.ArmM);
    tn = tn+robot.dt;
    if tn>toc(st)
        pause(tn-toc(st))
    end
end

disp('done.');
end