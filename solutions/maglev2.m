%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6016, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.02;
n_samples = 400;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;

%% identification
z0 = 5
[sys, I0] = maglev_ident(z0);
%%
mode = OPEN_LOOP
set_mode_params(arduino, mode, I0, [I0, z0]);
set_disturbance(arduino, 0)
reset_system(arduino)
%% Classical
sysd = c2d(sys, T_sample);
ps = pole(sysd)
prop_factor = 0.8;
R = zpk([ps(2)], [prop_factor*ps(2)], 1, T_sample);
figure; rlocus(-R*sysd);
[r, ks] = rlocus(-R*sysd);
idx = find(imag(r(3,:)) ~= 0, 1);
k1 = -ks(idx);
%
sys_cl = feedback(k1*R*sysd, 1);
figure; step(sys_cl);
figure; step(sys_cl/sysd);
%%
% Second loop: zero static error
ps2 = real(pole(sys_cl));
R2 = zpk([ps2(2)], [1], 1, T_sample);
figure; rlocus(R2*sys_cl);
[r, ks] = rlocus(R2*sys_cl);
idx = find(imag(r(3,:)) > 0, 1);
k2 = ks(idx)
%k2 = 0.155
%%
sysd_cl2 = feedback(k2*R2*sys_cl, 1);
figure; step(sysd_cl2)
figure; step(sysd_cl2/sysd)
%%
mode = CLASSICAL;
reset_system(arduino);
set_disturbance(arduino, 0.0);
w = 0;
set_mode_params(arduino, mode, w, [k1, ps(2), prop_factor*ps(2), k2, ps2(2)]);
input('press enter')

w = 1.0;
Y = get_response(arduino, w, n_samples)
u = Y(1, :); y = Y(2, :);
%%
y_ = step(sysd_cl2, ts)*w;
figure; hold on
plot(ts, y); title("y");
plot(ts, y_)
%
u_ = step(sysd_cl2/sysd, ts)*w;
figure; hold on
plot(ts, u); title("u");
plot(ts, u_ + I0)
%% Design observer
rank(obsv(sys.A, sys.C)) == 2
sysd = c2d(sys, T_sample)
ps = [0.15, 0.16]; L = place(sysd.A', sysd.C', ps); L=L';
y_ = step(sysd, ts);

% design state-space controller
ps = [0.99 0.991];
K = place(sysd.A, sysd.B, ps);
sysd_cl = ss(sysd.A-sysd.B*K, sysd.B, [sysd.C; -K], [sysd.D; 1], T_sample, ...
            'InputName', {'w'}, 'OutputName', {'z', 'I'});
final_value = sysd_cl.C(1, :)*inv(eye(2)-sysd_cl.A)*sysd_cl.B;
w = 1.0/final_value;
figure; step(w*sysd_cl)
%%
reset_system(arduino);
set_disturbance(arduino, 0.0);
mode = STATE_SPACE;
w = 0.0;
set_mode_params(arduino, mode, w, [K(1), K(2), L(1), L(2)])

input('press enter')

w = 2.0/final_value;
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%
%%
y_ = w*step(sysd_cl, ts);
figure;
subplot(311);
plot(ts, y, ts, x, ts, y_(:, 1));
legend('measured', 'estimated', 'simulated')
title("\theta"); xlabel('t')
subplot(312);
plot(ts, x_dot); title("\omega"); xlabel('t')
subplot(313);
plot(ts, u, ts, y_(:, 2)); title("I"); xlabel('t')
legend('measured', 'simulated')

%% Design observer
rank(obsv(sys.A, sys.C)) == 2
sysd = c2d(sys, T_sample)
ps = [0.25, 0.16]; L = place(sysd.A', sysd.C', ps); L=L';
y_ = step(sysd, ts);
%% Extended feedback
AE = [sysd.A, zeros(2,1); sysd.C, 1];
BE = [sysd.B; 0];
ps = [0.88, 0.889, 0.89];
KE = place(AE, BE, ps)

sysd_cl = ss(sysd.A-sysd.B*KE(1:2), sysd.B, sysd.C, 0, T_sample);
z = tf('z', T_sample);
RI = KE(3)/(z-1);
sysE_cl = feedback(RI*sysd_cl, 1);
figure; step(sysE_cl)
%%
mode = EXTENDED;
w = 0.0;
reset_system(arduino);
set_mode_params(arduino, mode, w, [KE(1), KE(2), KE(3), L(1), L(2)])
input('press enter')

w = 2
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
y_ = w*step(sysE_cl, ts);
figure;
subplot(211);
plot(ts, y, ts, x, ts, y_);
legend('measured', 'estimated', 'simulated')
title("z"); xlabel('t')
subplot(212);
plot(ts, x_dot); title("v"); xlabel('t')

%%
close_connection(arduino)
clear arduino