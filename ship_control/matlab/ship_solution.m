if 1==1

T=Time(n_period_before+1:end);
T=T-T(1);
step_response=theta(n_period_before+1:end);
figure;hold;plot(T,theta0,'b');
[Ks,s_pole,t0,err,time,step_response_calc]=second_order_identify(Tsample,step_response);
plot(T,step_response_calc,'r');

den=conv([-1/s_pole 1],[-1/conj(s_pole) 1]);
sys=tf(Ks,den);
step_response_calc1=step(sys,T);
plot(T,step_response_calc1,'g');
end
sysd=c2d(sys,Tsample);
figure;
rlocus(sysd);

figure;
KS=[0:.01:8];
rlocus(sysd,KS);

K=3.92;
premult=1/(K*Ks/(1+K*Ks));
w=0.1;
w_corr=w*premult;
sysP=feedback(K*sysd,1);
sysPu=feedback(K,sysd);

figure;step(sysP*w_corr);
figure;step(sysPu*w_corr);
