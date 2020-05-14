%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6012, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.05;
n_samples = 400;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%% identification
z0 = 5; % equilibrium point
[sys, I0] = maglev_ident(z0);
sysd = c2d(sys, T_sample)
%%
mode = OPEN_LOOP
set_mode_params(arduino, mode, I0, [I0, z0]); % use to set equilibrium current & position
set_disturbance(arduino, 0)
reset_system(arduino)

%% Design observer & controller
L = [0.0; 0.0] % to compute
K = [0.0, 0.0] % to compute
mode = STATE_SPACE
w = 0.0
set_mode_params(arduino, mode, w, [K(1), K(2), L(1), L(2)]);