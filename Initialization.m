defineTrajectory;

%% Initialization
t_final = 100;     % simulation time
dt = 1.00e-03;    % time-steps for solver

% initialize states
dq_0 = [0; 0; 0];  % initial conditions for state velocities

% Transformation from R2 to R1
rod_length = [0.20;0;0]; % meters

R1_T_R2 = [-1,  0,  0, P2_R1_pose(1)*2 + rod_length(1);
            0, -1,  0,                            0;
            0,  0,  1,                            0;
            0,  0,  0,                            1];

R2_T_R1 = [-1,  0,  0, P2_R1_pose(1)*2 + rod_length(1);
            0, -1,  0,                            0;
            0,  0,  1,                            0;
            0,  0,  0,                            1];