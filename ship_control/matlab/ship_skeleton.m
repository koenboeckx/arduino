arduino_port='/dev/ttyACM0';%define appropriate port here
%define controller mode (to be sent to the arduino in get_response)
OPEN_LOOP=0;
CLASSICAL_CURRENT=1;%classical (PID family) current controller
CLASSICAL_SPEED=2;%classical (PID family) speed controller
STATE_SPACE=3;
%-------------------------------------------------------------
Tsample=20E-3;downsample=1;
mode=OPEN_LOOP;
%mode=CLASSICAL_CURRENT;
n_meas_per_period=2;
n_period=1000;
n_period_before=50;
Time=[0:n_period-1]'*Tsample*downsample;
if ~exist('arduino','var')
   arduino=init_serial(n_meas_per_period,n_period,arduino_port);
   arduino.n_period_before=n_period_before;
    arduino.downsample=downsample;
end
%premult = 1.
%w=0.1*premult;
w=1
w0=0;
params=[];%use params=[] if no parameters are needed
err=set_mode_param(arduino,mode,w0,params);
input('type enter to continue');
Y=get_response(arduino,w);
w=Y(:,1);theta=Y(:,2);
figure;hold;plot(Time,w,'b');plot(Time,theta,'r');title('Open loop');legend({'w';'theta'});
err=set_mode_param(arduino,mode,0,params);

%%
s_real = 0.1; s_imag = 0.2; 
y = exponential(Time, s_real, s_imag, -0.2);
figure; plot(Time, y);