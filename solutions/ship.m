%% define parameters and create tcp/ip object
arduino = tcpclient('localhost', 6017, 'Timeout', 60);

%% define parameters and modes
T_sample = 0.05;
n_samples = 200;
ts = (0:n_samples-1)*T_sample;

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP   = 0;
CLASSICAL   = 1; %classical (PID family) controller
STATE_SPACE = 2;
EXTENDED    = 3;
TEST        = 4;

%%
mode = OPEN_LOOP;
w = -1.0;
set_mode_params(arduino, mode, w, [])
input('press enter')

w = 1.0
Y = get_response(arduino, w, n_samples);
u = Y(1, :); y = Y(2, :); 
%%
figure; plot(ts, y); title("y"); xlabel('t')
figure; plot(ts, u); title("u"); xlabel('t')
%%
set_mode_params(arduino, OPEN_LOOP, 0.0, []);
reset_system(arduino);
%%
n_samples = 400;
ts = (0:n_samples-1)*T_sample;
%%
set_disturbance(arduino, asin(1.0));
pause(1.0);
set_disturbance(arduino, 0.);
Y = get_response(arduino, 0.0, n_samples);
u = Y(1, :); y = Y(2, :); 
figure; plot(ts, y); title("y"); xlabel('t')
%% Estimate parameters
%load ship.mat
fun = @(x, t) exp(-x(1).*x(2)*t) .* cos(x(2).*sqrt(1 - (x(1).^2))*t);
x0 = [1, 1];
x_est = lsqcurvefit(fun, x0, ts, y);
%
figure; hold on;
plot(ts, y); title("y"); xlabel('t');
plot(ts, fun(x_est, ts));
%%
set_mode_params(arduino, OPEN_LOOP, 0.0, []);
reset_system(arduino);
w = 1.0;
Y = get_response(arduino, w, n_samples);
u = Y(1, :); y = Y(2, :); 
final_value = mean(y(end-10:end));
%%
eta = x_est(1); wn = x_est(2);
K1 = wn^2;
Cr = eta*2*wn;
K2 = K1 * final_value;
%%
A = [0 1;-K1 -Cr];
B = [0; K2];
C = [1 0];
D = 0;
sys = ss(A,B,C,D)
%%
poles = [-w*eta + 1j* wn*sqrt(1-eta^2), -w*eta - 1j* wn*sqrt(1-eta^2)];
H = zpk([], poles, poles(1)*poles(2));

%%
%final_value = 0.1717;
figure;hold on
plot(ts, y);
y_ = step(sys, ts);
plot(ts, y_);
%%
%sys  = H*final_value;
sysd = c2d(tf(sys), T_sample); 
figure; rlocus(-sysd);
%% First Classical Controller
k1 = 5;
sysd_wy = feedback(-k1*sysd, 1);
sysd_wu = feedback(-k1, sysd);
figure; step(sysd_wy, ts);
%%
ps = sort(pole(sysd_wy));
R = zpk([ps(1)], [0.8*ps(1)], 1, T_sample);
figure; rlocus(-R*sysd_wy);
k2 = .91;
sysd_wy2 = feedback(-k2*R*sysd_wy, 1);
figure; step(sysd_wy2);
figure; step(sysd_wy2/sysd)
%%
ps2 = sort(pole(sysd_wy2));
R = zpk([real(ps2(2))], [1], 1, T_sample);
figure; rlocus(R*sysd_wy2);
k3 = 0.19;
sysd_wy3 = feedback(k3*R*sysd_wy2, 1);
figure; step(sysd_wy3);
figure; step(sysd_wy3/sysd)
%%
mode = CLASSICAL;
w = 0.0;
%reset_system(arduino);
set_mode_params(arduino, mode, w, [-k1, -k2, ps(1), 0.8*ps(1), k3, real(ps2(2))]);

input('press enter')

w = 0.15;
Y = get_response(arduino, w, n_samples);
u = Y(1, :); y = Y(2, :); 
%%
figure; plot(ts, u); title("u"); xlabel('t')

figure; hold on
plot(ts, y); title("y"); xlabel('t')
y_ = w*step(sysd_wy3, ts);
plot(ts, y_);
%%
figure; hold on
plot(ts, y); title("y");
y_ = w*step(sysd_wy2, ts);
plot(ts, y_);
%%
figure; hold on
plot(ts, u); title("u");
u_ = w*step(sysd_wu, ts);
plot(ts, u_);
%% Alternative Classical Controller
sysd = c2d(sys, T_sample)
R = zpk(pole(sysd), [1 .8 0], 1, T_sample);
figure; rlocus(R*sysd)
[r, ks] = rlocus(R*sysd);
idx = find(imag(r(3,:)) ~= 0, 1);
k = ks(idx);
sysd_cl = feedback(k*R*sysd, 1);
figure; step(sysd_cl);
figure; step(sysd_cl/sysd);
%%
arduino = tcpclient('localhost', 6016, 'Timeout', 60);
%%
mode = TEST
w0 = 0.0
set_mode_params(arduino, mode, w0, [k]);
input('press enter')

w = 0.15;
Y = get_response(arduino, w, n_samples);
u = Y(1, :); y = Y(2, :); 
%
y_ = step(sysd_cl, ts)*w;
figure; plot(ts, y, ts, y_);

%% Design Observer
rank(obsv(sys.A, sys.C)) == 2
sysd = c2d(sys, T_sample);

ps = [0.75, 0.76]; L = place(sysd.A', sysd.C', ps); L=L';
y_ = step(sysd, ts);


%% design state-space controller
ps = [0.92 0.93];
K = place(sysd.A, sysd.B, ps);
sysd_cl = ss(sysd.A-sysd.B*K, sysd.B, [sysd.C; -K], [sysd.D; 1], T_sample, ...
            'InputName', {'sin(\phi)'}, 'OutputName', {'\theta', 'sin(\phi)'});
final_value = sysd_cl.C(1, :)*inv(eye(2)-sysd_cl.A)*sysd_cl.B;
w = 0.15/final_value;
figure; step(w*sysd_cl)

%%
mode = STATE_SPACE;
w0 = 0.0;

set_mode_params(arduino, mode, w0, [K(1), K(2), L(1), L(2)])
reset_system(arduino);
input('press enter')

Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%
%
y_ = w*step(sysd_cl, ts);
figure;
subplot(311); plot(ts, y, ts, y_(:, 1), ts, x); title('y')
subplot(312); plot(ts, u); title('u')
subplot(313); plot(ts, x_dot); title('\omega')


%% Extended feedback
sysd = c2d(sys, T_sample);
AE = [sysd.A, zeros(2,1); sysd.C, 1];
BE = [sysd.B; 0];
%ps = [0.90, 0.91, 0.92];
%ps = [0.88, 0.87, 0.86];
ps = [0.88, 0.89, 0.90];
KE = place(AE, BE, ps)

sysd_cl = ss(sysd.A-sysd.B*KE(1:2), sysd.B, sysd.C, 0, T_sample);
z = tf('z', T_sample);
RI = KE(3)/(z-1);
sysE_cl = feedback(RI*sysd_cl, 1);
figure; step(sysE_cl)
%
mode = EXTENDED;
w = 0.0;
%reset_system(arduino);
set_mode_params(arduino, mode, w, [KE(1), KE(2), KE(3)])
input('press enter')

w = 10*pi/180
Y = get_response(arduino, w, n_samples);
%
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
y_ = w*step(sysE_cl, ts);
u_ = w*step(tf(sysE_cl)/tf(sysd), ts);

figure;
subplot(211);
plot(ts, y, ts, x, ts, y_);
legend('measured', 'estimated', 'simulated')
title("\theta"); xlabel('t')
subplot(212);
plot(ts, x_dot); title("\omega"); xlabel('t')
figure; plot(ts, u, ts, u_);
%%
w = -10*pi/180
Y = get_response(arduino, w, n_samples);
y = Y(2, :); u = Y(1, :);
x = Y(3, :); x_dot = Y(4, :);
%%
close_connection(arduino)
clear arduino
