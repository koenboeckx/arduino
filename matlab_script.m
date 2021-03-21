%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6014, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.05;
n_samples = 200;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%%
mode = OPEN_LOOP;
w = -1.0;
set_mode_params(arduino, mode, w, [])
input('press enter')

w = 1.0
Y = get_response(arduino, w, n_samples)

%%
u = Y(1,:), y = Y(2,:)
figure; plot(ts, y); title("y"); xlabel('t')
figure; plot(ts, u); title("u"); xlabel('t')

%% Estimate parameters

p = polyfit(ts(1:43), y(1:43), 2);
x_fit = polyval(p, ts);

figure; hold on;
plot(ts, y, ts, x_fit);

%%
mode = CLASSICAL;
w = 0.0;
set_mode_params(arduino, mode, w, [2.]);

input('press enter')

w = 0.1
Y = get_response(arduino, w, n_samples)
u = Y(1,:), y = Y(2,:);
figure; plot(ts, y); title("y");
figure; plot(ts, u); title("u");

%%
close_connection(arduino)
clear arduino
