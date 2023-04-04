% Goal is to define a trajectory in task-space.
% Robot Definition from URDF
eeName = 'Link_EE';

pathToURDF.robot = './3DoFRobot.urdf';
robot = importrobot(pathToURDF.robot);

numJoints = numel(robot.homeConfiguration);
robot.homeConfiguration.JointName

% initial configuration is given
q_0 = [0;0;0];
initPosition = FwdKin(q_0);

% maximum number of waypoints
maxWaypoints = 20;

% Positions (X Y Z)
a = 0.25;
waypoints = initPosition + ...
    [  0,   0,   0;
       0,  -a,  -a;
       0,   0,-2*a;
       0,   a,  -a;
       0,   0,   0]';

numWaypoints = size(waypoints, 2);

% Array of waypoint times
waypointTimes = 0:4:16;

% Trajectory Sample Time
% These will be all the calculated points for the robot to hit (even in
% between waypoints)
ts = 0.2;
trajTimes = 0:ts:waypointTimes(end);

%% Additional Parameters

% Boundary conditions (for polynomial trajectories)
% Velocity (cubic and quintic)
waypointVels = 0.1*[ 0, -1,  0;
                     0,  0, -1;
                     0,  1,  0;
                     0,  0,  1;
                     0, -1,  0]';

% Acceleration (quintic only)
waypointAccels = zeros(size(waypointVels));

% Acceleration times (trapezoidaly only)
waypointAccelTimes = diff(waypointTimes)/4;

disp('PUMA Task Space Trajectory Generation and Evaluation')
tic
trajType = 'quintic';
switch trajType
    case 'trap'
        [posTask,velTask,accelTask] = trapveltraj(waypoints,numel(trajTimes), ...
            'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
            'EndTime',repmat(diff(waypointTimes),[3 1]));
    case 'cubic'
        [posTask,velTask,accelTask] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels);
        
    case 'quintic'
        [posTask,velTask,accelTask] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
            'VelocityBoundaryCondition',waypointVels, ...
            'AccelerationBoundaryCondition',waypointAccels);
        
    case 'bspline'
        ctrlpoints = waypoints; % Can adapt this as needed
        [posTask,velTask,accelTask] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
        
    otherwise
        error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
end


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