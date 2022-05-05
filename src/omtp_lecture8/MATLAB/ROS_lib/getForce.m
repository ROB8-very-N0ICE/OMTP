function [Fm] = getForce(robot)
%% get forces 
% robot.Fs = rossubscriber("/panda_finger1/ft_sensor_topic");
robot.FsData = receive(robot.Fs,3);

Fm(1) = robot.FsData.Wrench.Force.X;
Fm(2) = robot.FsData.Wrench.Force.Y;
Fm(3) = robot.FsData.Wrench.Force.Z;

Fm(4) = robot.FsData.Wrench.Torque.X;
Fm(5) = robot.FsData.Wrench.Torque.Y;
Fm(6) = robot.FsData.Wrench.Torque.Z;