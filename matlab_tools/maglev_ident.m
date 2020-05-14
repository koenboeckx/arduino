function [sys, I0] = maglev_ident(z0)
% z0 is the required equilibrium point
% returns the identified system 'sys' and the equilibrium current 'I0'

mass = 0.5; g=9.81; k=2.5;
z0s = 0:9;
I0s = mass*g/k * (z0s+1);
ps = polyfit(z0s, I0s, 2);

idx = find(z0s == z0, 1);
I0 = I0s(idx);
h = 0.01;

Il = polyval(ps, z0-h);
Ih = polyval(ps, z0+h);
f_deriv = (g/Ih-g/Il)/(2*h);

%
A = [0 1; -I0*f_deriv, 0];
B = [0; -g/I0]; C = [1, 0];
sys  = ss(A, B, C, 0);

