function err=set_mode_param(arduino,mode,w0,params)
%this function communicates with the arduino and calls the function set_mode_param (to be written in the .ino file) on the arduino 
%arduino: a structure containing the com object and other response parameters created by init_serial
%mode:the mode for which the the parameters should be set
%w0: the initial set point to use when setting that mode
%params: the parameters (float array) tranmitted to the arduino
%        if params=[], the function set_mode_param is not called on the arduino. Only set the mode and w0, keeping the existing parameters
params=params(:);
n_param=length(params);
%disp('set_mode_param: Writing to serial port');
fwrite(arduino.com,mode,'uint32');
fwrite(arduino.com,w0,'float');
fwrite(arduino.com,n_param,'uint32');
%pause(2)
%f (get(arduino.com, 'BytesAvailable') < 16) 
%   disp(['set_mode_param: No response from Arduino board' num2str(get(arduino.com, 'BytesAvailable'))]);
%   keyboard
%end
mode_echo=fread(arduino.com,1,'uint32');
if (mode_echo~=mode)
    error('COM error for mode');
end

w0_echo=fread(arduino.com,1,'float');
if (abs(w0_echo-w0)>1E-5)
    error('COM error for w0');
end
n_param_echo=fread(arduino.com,1,'uint32');
if (n_param_echo~=n_param)
    error('COM error for n_param');
end

if n_param>0
    fwrite(arduino.com,params,'float');
    params_echo=fread(arduino.com,n_param,'float');
        if (max(abs(params_echo-params))>1E-5)
            error('COM error for params');
        end
end
%disp('Before Parameters successfully set !!');
err=fread(arduino.com,1,'uint32');
%disp(['Parameters successfully set !! err = ' num2str(err)]);
