% Goal is to define a trajectory in task-space.
% Robot Definition from URDF
eeName = 'Link_EE';

pathToURDF.robot = './3DoFRobot.urdf';

% Robot 1
robot1 = importrobot(pathToURDF.robot);
numJoints = numel(robot1.homeConfiguration);
robot1.homeConfiguration.JointName
% Robot 2
robot2 = importrobot(pathToURDF.robot);
numJoints = numel(robot2.homeConfiguration);
robot2.homeConfiguration.JointName


% change home configuration
ikInitGuess = [0;-pi/1.999;4*pi/3];
ikInitGuessSim = ikInitGuess; % initial configuration of both R1 and R2
initPosition = FwdKin(ikInitGuess);

homeConfig = struct('JointName', 'Joint_1');
homeConfig(2).JointName = 'Joint_2';
homeConfig(3).JointName = 'Joint_3';
homeConfig(1).JointPosition = ikInitGuess(1);
homeConfig(2).JointPosition = ikInitGuess(2);
homeConfig(3).JointPosition = ikInitGuess(3);
ikInitGuess = homeConfig;

% maximum number of waypoints
maxWaypoints = 20;

%% Phase 1 Trajectories

% Robot 1
% Positions (X Y Z)
P1_R1_waypoints = horzcat(initPosition,...
    [0.4000, 0, 0.8500;
     0.7395, 0, 0.9786]');
% Velocity (cubic and quintic)
P1_R1_waypointVels = 0.1*[    1.5,  0,   1;
                          3*0.5/2,  0, 1/2;
                                0,  0,   0]';
numWaypoints = size(P1_R1_waypoints, 2);
% Array of waypoint times (offset to be added to simulation time)
P1_R1_waypointTimes = 0:3:6;
% Trajectory Sample Time
% These will be all the calculated points for the robot to hit (even in
% between waypoints)
ts = 0.2;
trajTimes = 0:ts:P1_R1_waypointTimes(end);

% Robot 2
% Just regulating home position with PD
% ikInitGuessSim

%% Phase 2
% Robot 1
P2_R1_pose = P1_R1_waypoints(end,:);

% Robot 2 - Desired trajectory off assumed success of Phase 1
% Relative to R2 base frame
P2_R2_waypoints = horzcat(initPosition,...
    [0.4000, 0, 0.8500;
     0.7395, 0, 0.9786]');
% Velocity (cubic and quintic)
P2_R2_waypointVels = 0.1*[    1.5,  0,   1;
                          3*0.5/2,  0, 1/2;
                                0,  0,   0]';
numWaypoints = size(P1_R1_waypoints, 2);
% Array of waypoint times (offset to be added to simulation time)
P2_R2_waypointTimes = 0:3:6;
% Trajectory Sample Time
% These will be all the calculated points for the robot to hit (even in
% between waypoints)
P2_R2_trajTimes = 0:ts:P2_R2_waypointTimes(end);


%% Trajectory Generation

disp('PUMA Task Space Trajectory Generation and Evaluation')
tic
[posTask,velTask,accelTask] = quinticpolytraj(P1_R1_waypoints,P1_R1_waypointTimes,trajTimes, ... 
    'VelocityBoundaryCondition',P1_R1_waypointVels, ...
    'AccelerationBoundaryCondition',waypointAccels);
taskTime = toc;

disp(['Task space trajectory generation time : ' num2str(taskTime) ' s']);
figure; hold on
plot3(posTask(1,:),posTask(2,:),posTask(3,:),'b-');
plot3(P1_R1_waypoints(1,:),P1_R1_waypoints(2,:),P1_R1_waypoints(3,:),'ko','LineWidth',2);
title('Trajectory Comparison'); 
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Task Space Trajectory','Waypoints');
grid on
view([45 45]);


% fwd kinematics
function [X] = FwdKin(Q)
    s1 = sin(Q(1, :));
    c1 = cos(Q(1, :));
    s2 = sin(Q(2, :));
    c2 = cos(Q(2, :));
    s23 = sin(Q(2, :) + Q(3, :));
    c23 = cos(Q(2, :) + Q(3, :));
    l = [0.6731; 0.432; 0.434];

    X(1, :) = c1.*(l(2)*c2 + l(3)*s23);
    X(2, :) = s1.*(l(2)*c2 + l(3)*s23);
    X(3, :) = l(1) - l(2)*s2 + l(3)*c23;
end