function Y=get_response_new(arduino,w)
%arduino: a structure containing the com object and other response parameters created by init_serial
%w (float): the set point to be send to the arduino
%Y : float array returned by the arduino. Size defined in init_serial.
if arduino.downsample<1
    warning('downsample shoule be >1');
    arduino.downsample=1;
end
if arduino.downsample>255
    warning('downsample shoule be <=255');
    arduino.downsample=255;
end

set(arduino.com,'BaudRate',arduino.BaudRateTX);
mode=255;%to trigger a response 
fwrite(arduino.com,mode,'uint32');
fwrite(arduino.com,arduino.n_period,'uint32');
fwrite(arduino.com,arduino.n_period_before,'uint32');
fwrite(arduino.com,arduino.downsample,'uint32');
fwrite(arduino.com,w,'float');
mode_echo=fread(arduino.com,1,'uint32');
if (mode_echo~=mode)
    error('COM error for mode');
end
n_period_echo=fread(arduino.com,1,'uint32');
if (n_period_echo~=arduino.n_period)
    error('COM error for n_period');
end
n_period_before_echo=fread(arduino.com,1,'uint32');
if (n_period_before_echo~=arduino.n_period_before)
    error('COM error for n_period_before');
end
downsample_echo=fread(arduino.com,1,'uint32');
if (downsample_echo~=arduino.downsample)
    error('COM error for downsample');
end
w_echo=fread(arduino.com,1,'float');
if (abs(w_echo-w)>1E-5)
    error('COM error for w');
end

set(arduino.com,'BaudRate',arduino.BaudRateRX);
disp('Waiting for response ....');
Y=fread(arduino.com,arduino.n_period*arduino.n_meas_per_period,'float');
Y=reshape(Y,[arduino.n_meas_per_period,arduino.n_period]);Y=Y';
set(arduino.com,'BaudRate',arduino.BaudRateTX);

