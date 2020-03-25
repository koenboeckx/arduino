%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6016, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.01;
n_samples = 200;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;

%%
mode = OPEN_LOOP;
w = 20.0;
height = 1.0;
set_disturbance(arduino, height)
set_mode_params(arduino, mode, w, [])
reset_system(arduino);
%%
set_mode_params(arduino, mode, w, [])
for w = linspace(20.0, 0.0, 101)
    [y, u] = get_response(arduino, w, 5);
    if(y(end) > height)
        break
    end
    pause(0.1)
end    
%%
mode = OPEN_LOOP;

w = -1.0
set_mode_params(arduino, mode, w, [])

input('press enter')

w = 1.0
[y, u] = get_response(arduino, w, n_samples)
%%
figure; plot(ts, y); title("y");
figure; plot(ts, u); title("u");

%% Estimate parameters

p = polyfit(ts(1:200), y(1:200), 2);
x_fit = polyval(p, ts);

figure; hold on;
plot(ts, y, ts, x_fit);

%%
% Define system
K = p(1)/(2*w);
S = tf([K/0.25], [1, 0, 0]);
y_ = step(S, ts);
plot(ts, y_+p(3))
%%
Sd = c2d(S, T_sample);
%figure; rlocus(Sd);
k1 = 1; Scl1 = feedback(k1*Sd, 1);
%figure; rlocus(Scl1);
ps = pole(Scl1);
R = zpk(ps(2), 0.9*ps(2), 1, T_sample);
figure; rlocus(-R*Scl1);

k2 = 168;
Scl2 = feedback(-k2*R*Scl1, 1);
figure; step(Scl2)
%%
mode = CLASSICAL;
w = 0.0;
set_mode_params(arduino, mode, w, [ps(2), 0.9*ps(2), -k2]);

input('press enter')

w = 0.05
n_samples = 300;
ts = (0:n_samples-1)*T_sample;
[y, u] = get_response(arduino, w, n_samples)
y_ = w*step(Scl2, ts);
figure; plot(ts, y, ts, y_); title("y");
figure; plot(ts, u); title("u");

%%
set_mode_params(arduino, mode, 0.0, [ps(2), 0.9*ps(2), -k2]);
%%
set_disturbance(0.1);
%%
close_connection(arduino)
clear CLASSICAL

%%
figure; hold on;
plot(ts, Y-Y(1), 'r')
step(0.5*sysd_cl)
