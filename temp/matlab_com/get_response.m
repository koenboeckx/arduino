function Y = get_response(arduino,w, readInPieces)
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
mode = 255;%to trigger a response 

fwrite(arduino.com,mode,'uint32');
fwrite(arduino.com,arduino.n_period,'uint32');
fwrite(arduino.com,arduino.n_period_before,'uint32');
fwrite(arduino.com,arduino.downsample,'uint32');
fwrite(arduino.com,w,'float');
%pause(2)
%if (get(arduino.com, 'BytesAvailable') < 20) 
%   disp('get_response: No response from Arduino board');
%   keyboard
%end
mode_echo=fread(arduino.com,1,'uint32');
if (mode_echo~=mode)
    disp('COM error for mode');
    keyboard
end
disp('Correct mode ....');
n_period_echo=fread(arduino.com,1,'uint32');
if (n_period_echo~=arduino.n_period)
    disp('COM error for n_period');
    keyboard
end
disp('Correct n_period ....');
n_period_before_echo=fread(arduino.com,1,'uint32');
if (n_period_before_echo~=arduino.n_period_before)
    disp('COM error for n_period_before');
    keyboard
end
disp('Correct n_period_before ....');
downsample_echo=fread(arduino.com,1,'uint32');
if (downsample_echo~=arduino.downsample)
    error('COM error for downsample');
end
disp('Correct downsample_echo ....');
w_echo=fread(arduino.com,1,'float');
if (abs(w_echo-w)>1E-5)
    error('COM error for w');
end
disp('Correct w_echo ....');

set(arduino.com,'BaudRate',arduino.BaudRateRX);
disp('Waiting for response ....');

if( exist( 'readInPieces', 'var'))
  keyboard
  Y=fread(arduino.com,arduino.n_period*arduino.n_meas_per_period,'float');
else
  Y=fread(arduino.com,arduino.n_period*arduino.n_meas_per_period,'float');
  Y=reshape(Y,[arduino.n_meas_per_period,arduino.n_period]);Y=Y';
  %Y=0;
end
set(arduino.com,'BaudRate',arduino.BaudRateTX);

