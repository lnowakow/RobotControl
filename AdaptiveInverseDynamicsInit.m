%% Adaptive Inverse Dynamics Init

l1 = 0.6731;
l2 = 0.432;
l3 = 0.434;
lc2 = 0.216;
lc3 = 0.164;
m2 = 15.46;
m3 = 9.55;
J = 0.302;
g = 9.81;

%% Fake Initial guess of parameters

lc2 = lc2*1.05;
lc3 = lc3*0.95;

%% Initial Parameter Estimation

p1 = J;
p2 = lc3^2 * m3 * g;
p3 = (lc2^2 * m2 + l2^2 * m3) * g;
p4 = l2 * lc3 * m3;
p5 = lc3 * m3 * g;
p6 = (lc2 * m2 + l2 * m3) * g;

theta_params_init = [p1;p2;p3;p4;p5;p6];

%% Kp and Kd controller parameters
Kp = 20*eye(3,3);
Kd = 50*eye(3,3);

%% P Matrix lyap
A = [zeros(3,3), eye(3,3);
           -Kp,      -Kd];
Q = 0.5*eye(size(A));
P = lyap(A', Q);