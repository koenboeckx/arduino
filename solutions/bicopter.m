%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6012, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.05;
n_samples = 100;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%%
mode = OPEN_LOOP;
w = -1;
set_mode_params(arduino, mode, w, [])
input('press enter')

w = 0.05
Y = get_response(arduino, w, n_samples);
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
figure;
subplot(211);
plot(ts, y, ts, x); title("\theta"); xlabel('t')
subplot(212);
plot(ts, x_dot); title("\omega"); xlabel('t')
%figure; plot(ts, u); title("u"); xlabel('t')
%%

%% Estimate parameters

n_max = find(y<-0.17, 1);
p = polyfit(ts(1:n_max), y(1:n_max), 2);
x_fit = polyval(p, ts);

figure; hold on;
plot(ts, y, ts, x_fit);
%
k_system = 2*p(1)/w;
%k_system = -1.4339;
sys = tf([k_system], [1, 0, 0]);
y_ = step(sys,ts) * w + y(1);
plot(ts, y_, ts, x_fit);

sysd = c2d(sys, T_sample);
%% Double feedback loop: Separate poles, cancel the stable one with LL
figure; rlocus(sysd);
k1 = 8;
sysd_cl1 = feedback(k1*sysd, 1);
ps = pole(sysd_cl1);
R = zpk([ps(2)], [0.6*ps(2)], 1, T_sample);
figure; rlocus(-R*sysd_cl1);
%%
k2 = -4.1;
sysd_cl2 = feedback(k2*R*sysd_cl1, 1);
figure; step(sysd_cl2);
sysd_wu = sysd_cl2/sysd;
figure; step(sysd_wu);
%% Third loop: zero static error
ps2 = sort(pole(sysd_cl2));
R = zpk([ps2(2)], [1], 1, T_sample);
figure; rlocus(R*sysd_cl2);
k3 = 0.09;
sysd_cl3 = feedback(k3*R*sysd_cl2, 1);
figure; step(sysd_cl3);
sysd_wu = sysd_cl3/sysd;
figure; step(sysd_wu);

%%
mode = CLASSICAL;
w = 0.0;
set_mode_params(arduino, mode, w, [k1, k2, ps(2), 0.6*ps(2), k3, ps2(3)]);

input('press enter to continue')

w = 10*pi/180;
Y = get_response(arduino, w, n_samples);
u = Y(1, :); y = Y(2, :);
%%
figure; hold on
plot(ts, y); title("y");
y_ = w*step(sysd_cl3, ts);
plot(ts, y_);
figure; plot(ts, u); title("u");
%%
set_mode_params(arduino, mode, 0.0, [k1, k2, ps(2), 0.6*ps(2), k3, ps2(3)]);
input('press enter to continue')
set_disturbance(arduino, 1.0);
input('press enter to continue')
set_disturbance(arduino, 0.0);
%% design observer
A = [0, 1; 0 0]; B = [0; k_system]; C = [1 0]; D=0;
rank(obsv(A, C)) == 2
sys = ss(A, B, C, D);
sysd = c2d(sys, T_sample);

ps = [0.95, 0.96]; L = place(sysd.A', sysd.C', ps); L=L';
y_ = step(sysd, ts);

%% design state-space controller
ps = [0.75 0.76];
K = place(sysd.A, sysd.B, ps);
sysd_cl = ss(sysd.A-sysd.B*K, sysd.B, sysd.C, sysd.D, T_sample);
final_value = sysd_cl.C*inv(eye(2)-sysd_cl.A)*sysd_cl.B;
%%
arduino = tcpclient('localhost', 6014, 'Timeout', 60);
%%
mode = STATE_SPACE;
w = 0.0;
set_mode_params(arduino, mode, w, [K(1), K(2), L(1), L(2)])
input('press enter')

w = 9*pi/180/final_value;
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
y_ = w*step(sysd_cl, ts);
subplot(211);
plot(ts, y, ts, x, ts, y_);
legend('measured', 'estimated', 'simulated')
title("\theta"); xlabel('t')
subplot(212);
plot(ts, x_dot); title("\omega"); xlabel('t')
%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6024, 'Timeout', 60);

%% Extended feedback
AE = [sysd.A, zeros(2,1); sysd.C, 1];
BE = [sysd.B; 0];
ps = [0.8, 0.81, 0.82];
KE = place(AE, BE, ps)

sysd_cl = ss(sysd.A-sysd.B*KE(1:2), sysd.B, sysd.C, 0, T_sample);
z = tf('z', T_sample);
RI = KE(3)/(z-1);
sysE_cl = feedback(RI*sysd_cl, 1);
figure; step(sysE_cl)
%
mode = EXTENDED;
w = 0.0;
reset_system(arduino);
%%
set_mode_params(arduino, mode, w, [KE(1), KE(2), KE(3), L(1), L(2)])
get_response(arduino, 0, 10);
input('press enter')

w = 5*pi/180
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
Y2=Y;
%%
y_ = w*step(sysE_cl, ts);
figure;
subplot(311);
plot(ts, y, ts, x, ts, y_);
legend('measured', 'estimated', 'simulated')
title("\theta"); xlabel('t')
subplot(312);
plot(ts, x_dot); title("\omega"); xlabel('t')
subplot(313);
plot(ts, u); title("u"); xlabel('t')
%% Extended with proportional term
z_PI = ps(3);
Ki = KE(3);
Kp = Ki/(1-z_PI);
Kcorr = KE(1:2)-Kp*Cd;
sysd_cl = ss(sysd.A-sysd.B*Kcorr, sysd.B, sysd.C, 0, T_sample);
Rpi = Kp + Ki/(z-1);
sysE_cl2 = feedback(Rpi*sysd_cl, 1);
figure; step(sysE_cl2)
%%
set_mode_params(arduino, mode, w, [Kcorr(1), Kcorr(2), Ki, L(1), L(2), Kp])
get_response(arduino, 0, 10);
input('press enter')

w = 5*pi/180
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
y_ = w*step(sysE_cl2, ts);
figure;
subplot(311);
plot(ts, y, ts, x, ts, y_);
legend('measured', 'estimated', 'simulated')
title("\theta"); xlabel('t')
subplot(312);
plot(ts, x_dot); title("\omega"); xlabel('t')
subplot(313);
plot(ts, u); title("u"); xlabel('t')
%%
set_disturbance(arduino, 1.0);
%%
w = 3*pi/180;
for i = 1:20
    get_response(arduino, w, 50);
    w = -w;
end
%%
close_connection(arduino)
clear arduino
