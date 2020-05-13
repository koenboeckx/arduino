function [sys, I0] = maglev_ident(arduino, z0)
% z0 is the required equilibrium point
% returns the identified system 'sys' and the equilibrium current 'I0'
%% define parameters and create tcp/ip object
%arduino = tcpclient('localhost', 6016, 'Timeout', 60);


%% identification

% mode = 0 % OPEN_LOOP
% 
% z0s = 0:9
% results = []
% for z0 = z0s
%     reset_system(arduino);
%     set_disturbance(arduino, z0);
%     set_mode_params(arduino, mode, 20, []);
%     input('press enter to continue')
%     for w = 20:-0.1:0.0
%         Y = get_response(arduino, w, 10);
%         y = Y(2, :);
%         if any(y > z0)
%             break
%         end
%     end
%     results = [results, w]
% end
% I0s = results;
%%
Im   = 0.0;
mass = 0.5;
g    = 9.81;
k    = 2.5;
%%

z0s = 0:9;
I0s = [3.7000    6.4000    8.8000   10.9000   12.8000   14.4000   16.0000   17.4000   18.6000   19.8000];
%I0s = [0    1.9620    3.9240    5.8860    7.8480    9.8100   11.7720   13.7340   15.6960   17.6580];
%I0s = mass*g/k * z0s;
ps = polyfit(z0s, I0s, 2);

figure; 
plot(z0s, I0s, z0s, polyval(ps, z0s));

%%
idx = find(z0s == z0, 1)
I0 = I0s(idx);
g = 9.81;

f_z0 = g/I0;
h = 0.01;

Il = polyval(ps, z0-h);
Ih = polyval(ps, z0+h);
f_deriv = (g/Ih-g/Il)/(2*h)

%
A = [0 1; -I0*f_deriv, 0];
B = [0; -g/I0]; C = [1, 0];
sys = ss(A, B, C, 0);
%%
%close_connection(arduino)
%clear arduino