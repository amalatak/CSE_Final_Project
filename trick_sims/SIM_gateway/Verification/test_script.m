clear; clc;

% Initial Position
time = 500;
mu_moon = 4.9048695e12;
init_pos = [1900.0e3; 1000.0; 1000.0];
init_vel = [1; sqrt(mu_moon/init_pos(1)); 1];

% Test calculating inertial to LVLH and DCM2quat
init_LVLH_i = calculate_LVLH_i(init_pos, init_vel);
init_body_i = [1 0 0; 0 0 -1; 0 -1 0];
init_quat = dcm2quat(init_LVLH_i);

% test orbit propagator
tspan = [0:10:time];
rv0 = [init_pos; init_vel];
options = odeset('RelTol', 1e-10);
[~, pos] = ode45(@d_orbit, tspan, rv0, options);
disp(pos(end, :))

% test lvlh2body
final_lvlh = calculate_LVLH_i(pos(end, 1:3)', pos(end, 4:6)');
body_in_lvlh = init_body_i*transpose(final_lvlh);

% test qmult
q_test1 = [sqrt(1/2) -sqrt(1/2) 0 0];
q_test2 = [0 sqrt(1/2) 0 -sqrt(1/2)];
qmult(q_test1, q_test2);

% test calculate_qdot
wtrue = [1 -2 .5];
qtrue = [-0.5000; 0.5000; -0.5000; 0.5000];
w = [wtrue(1); wtrue(2); wtrue(3); 0];
qbartrue = .5*qmult(w, qtrue);

% test wdes if center pointing
wdes = cross(init_pos, init_vel)/norm(init_pos)^2;



