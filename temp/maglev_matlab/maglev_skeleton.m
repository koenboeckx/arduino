% Define Arduino serial port here
arduino_port='com4'; % For window
arduino_port='/dev/cu.usbmodem1411'; % For MacOs X
arduino_port='/dev/ttyUSB1'; % For Linux

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP  = 0;
CLASSICAL  = 1; %classical (PID family) controller
STATE_SPACE= 2;
% Define controller mode to be sent to the Arduino in get_response
mode = OPEN_LOOP;
%-------------------------------------------------------------
Tsample = 20E-3; downsample=1;
n_meas_per_period = 3;
n_period = 400;
n_period_before = 10;
Time=[0:n_period-1]'*Tsample*downsample;
if ~exist('arduino','var')
   arduino=init_serial(n_meas_per_period,n_period,arduino_port);
   arduino.n_period_before=n_period_before;
   arduino.downsample=downsample;
end

% Set adapted baudrate
arduino.BaudRateTX=9600;
arduino.BaudRateRX=115200;

heigth = 10 % maximum height of box

%% Measure step response
w0 = 2.5;
err = set_mode_param(arduino, mode, w0, [5.0]);
input('Press enter to continue')
w  = 2.501;

Y = get_response(arduino, w);
input = Y(:,1);
I     = Y(:,2);
z     = Y(:,3);

ts = (0:length(input)-1) * Tsample;
figure; hold on;
plot(ts, heigth-z);
plot(ts, I);